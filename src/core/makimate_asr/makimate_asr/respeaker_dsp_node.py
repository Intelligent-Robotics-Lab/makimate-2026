"""
respeaker_dsp_node.py — ReSpeaker USB 4-Mic Array DSP Configuration & Polling
===============================================================================
Configures the ReSpeaker's built-in XMOS DSP on startup and polls status
parameters at a fixed rate, publishing them as ROS 2 topics.

Published Topics:
  /respeaker/doa        std_msgs/Int32   — Direction of Arrival (0-359 degrees)
  /respeaker/vad        std_msgs/Bool    — Voice Activity Detection
  /respeaker/proximity  std_msgs/Float32 — Proximity estimate (0.0=far, 1.0=close)

Parameters:
  tuning_script_path  str    Path to tuning.py from usb_4_mic_array repo
  poll_rate_hz        float  How often to poll the device (default: 10 Hz)
  vad_threshold       float  GAMMAVAD_SR in dB (default: 3.5, higher = less sensitive)
  enable_ns           bool   Noise suppression for ASR (default: True)
  enable_echo         bool   Echo + non-linear echo suppression (default: True)
  enable_agc          bool   Automatic Gain Control (default: True)
  hpf_cutoff          int    High-pass filter: 0=off, 1=70Hz, 2=125Hz, 3=180Hz
"""

import math
import importlib.util

import usb.core
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool, Float32
from rcl_interfaces.msg import SetParametersResult


class ReSpeakerDSP(Node):

    def __init__(self):
        super().__init__('respeaker_dsp')

        # ------------------------------------------------------------------ #
        # Parameters
        # ------------------------------------------------------------------ #
        self.declare_parameter('tuning_script_path',
                               '/home/emanuel/usb_4_mic_array/tuning.py')
        self.declare_parameter('poll_rate_hz',   10.0)
        self.declare_parameter('vad_threshold',   3.5)   # dB — raise to reduce false triggers
        self.declare_parameter('enable_ns',      True)
        self.declare_parameter('enable_echo',    True)
        self.declare_parameter('enable_agc',     True)
        self.declare_parameter('hpf_cutoff',        2)   # 125 Hz cut-off

        tuning_path = self.get_parameter('tuning_script_path').value
        poll_rate   = float(self.get_parameter('poll_rate_hz').value)
        vad_thresh  = float(self.get_parameter('vad_threshold').value)
        enable_ns   = bool(self.get_parameter('enable_ns').value)
        enable_echo = bool(self.get_parameter('enable_echo').value)
        enable_agc  = bool(self.get_parameter('enable_agc').value)
        hpf_cutoff  = int(self.get_parameter('hpf_cutoff').value)

        # ------------------------------------------------------------------ #
        # Load Tuning class dynamically from usb_4_mic_array/tuning.py
        # ------------------------------------------------------------------ #
        try:
            # Find ReSpeaker USB device (Seeed vendor 0x2886, product 0x0018)
            usb_dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
            if usb_dev is None:
                raise RuntimeError(
                    'ReSpeaker not found. Check USB connection and udev rules.'
                )
            spec   = importlib.util.spec_from_file_location('tuning', tuning_path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            self._tuning = module.Tuning(usb_dev)
            self.get_logger().info(f'ReSpeakerDSP: loaded tuning module from {tuning_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load tuning module: {e}')
            self.get_logger().error(
                'Make sure usb_4_mic_array is cloned and udev rules are installed.'
            )
            raise

        # ------------------------------------------------------------------ #
        # Configure DSP on startup
        # ------------------------------------------------------------------ #
        self._configure_dsp(enable_ns, enable_echo, enable_agc,
                            hpf_cutoff, vad_thresh)

        # ------------------------------------------------------------------ #
        # Publishers
        # ------------------------------------------------------------------ #
        self._doa_pub       = self.create_publisher(Int32,   '/respeaker/doa',       10)
        self._vad_pub       = self.create_publisher(Bool,    '/respeaker/vad',       10)
        self._proximity_pub = self.create_publisher(Float32, '/respeaker/proximity', 10)

        # ------------------------------------------------------------------ #
        # Live parameter updates
        # ------------------------------------------------------------------ #
        self.add_on_set_parameters_callback(self._on_params)

        # ------------------------------------------------------------------ #
        # Polling timer
        # ------------------------------------------------------------------ #
        self.create_timer(1.0 / poll_rate, self._poll)
        self.get_logger().info(
            f'ReSpeakerDSP running at {poll_rate} Hz  |  '
            f'/respeaker/doa  /respeaker/vad  /respeaker/proximity'
        )

    # ====================================================================== #
    # Live parameter callback — applies changes immediately to DSP registers
    # ====================================================================== #

    def _on_params(self, params):
        _DSP_MAP = {
            'enable_agc':  [('AGCONOFF',             lambda v: 1 if v else 0)],
            'enable_ns':   [('STATNOISEONOFF_SR',    lambda v: 1 if v else 0),
                            ('NONSTATNOISEONOFF_SR', lambda v: 1 if v else 0)],
            'enable_echo': [('ECHOONOFF',            lambda v: 1 if v else 0),
                            ('NLATTENONOFF',         lambda v: 1 if v else 0)],
            'vad_threshold': [('GAMMAVAD_SR',        lambda v: float(v))],
            'hpf_cutoff':    [('HPFONOFF',           lambda v: int(v))],
        }
        for p in params:
            if p.name not in _DSP_MAP:
                continue
            for reg, fn in _DSP_MAP[p.name]:
                try:
                    val = fn(p.value)
                    self._tuning.write(reg, val)
                    self.get_logger().info(f'[live] DSP {reg} = {val}')
                except Exception as e:
                    self.get_logger().warn(f'[live] DSP write {reg} failed: {e}')
                    return SetParametersResult(successful=False, reason=str(e))
        return SetParametersResult(successful=True)

    # ====================================================================== #
    # DSP startup configuration
    # ====================================================================== #

    def _configure_dsp(self, ns: bool, echo: bool, agc: bool,
                       hpf: int, vad_thresh: float):
        settings = [
            ('STATNOISEONOFF_SR',    1 if ns   else 0),
            ('NONSTATNOISEONOFF_SR', 1 if ns   else 0),
            ('ECHOONOFF',            1 if echo else 0),
            ('NLATTENONOFF',         1 if echo else 0),
            ('AGCONOFF',             1 if agc  else 0),
            ('HPFONOFF',             hpf),
            ('GAMMAVAD_SR',          vad_thresh),
        ]
        for name, value in settings:
            try:
                self._tuning.write(name, value)
                self.get_logger().info(f'  DSP {name} = {value}')
            except Exception as e:
                self.get_logger().warn(f'  DSP write {name} failed: {e}')

    # ====================================================================== #
    # Polling callback — runs at poll_rate_hz
    # ====================================================================== #

    def _poll(self):
        try:
            # --- Direction of Arrival (0–359 degrees) ---
            doa_msg = Int32()
            doa_msg.data = int(self._tuning.read('DOAANGLE'))
            self._doa_pub.publish(doa_msg)

            # --- Voice Activity Detection ---
            vad_msg = Bool()
            vad_msg.data = bool(self._tuning.read('VOICEACTIVITY'))
            self._vad_pub.publish(vad_msg)

            # --- Proximity estimate via AGC gain ---
            # AGC gain:  1  (0 dB)  → loud / close  → proximity ≈ 1.0
            #            1000 (60 dB) → quiet / far  → proximity ≈ 0.0
            gain = max(1.0, float(self._tuning.read('AGCGAIN')))
            proximity = max(0.0, 1.0 - math.log10(gain) / 3.0)
            prox_msg = Float32()
            prox_msg.data = float(proximity)
            self._proximity_pub.publish(prox_msg)

        except Exception as e:
            self.get_logger().warn(
                f'Poll error: {e}', throttle_duration_sec=5.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = ReSpeakerDSP()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
