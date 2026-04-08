#!/usr/bin/env python3
import math
import time
from pathlib import Path
from typing import List

import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from rcl_interfaces.msg import SetParametersResult

from dynamixel_sdk import (
    PortHandler,
    PacketHandler,
    COMM_SUCCESS,
)

# 4096 ticks per 360 degrees
TICKS_PER_REV = 4096.0
DEG_PER_REV = 360.0
TICKS_PER_DEG = TICKS_PER_REV / DEG_PER_REV  # ~11.38 ticks/deg


# ----------------------------------------------------
# FALLBACK LIMITS — used only when no YAML config is found.
# Run tools/calibrate_motors.py to generate config/motor_limits.yaml
# for your specific robot.
# ----------------------------------------------------
_DEFAULT_ROBOT_LIMITS = {
    1: {"min": 28, "max": 570},
    2: {"min": 1748, "max": 2348},
    3: {"min": 532, "max": 1467},
    4: {"min": 1809, "max": 2241},
    5: {"min": 2432, "max": 3121},
    6: {"min": 1097, "max": 1786},
}
_DEFAULT_NEUTRAL_POSITIONS = {
    1: 460,   # eye_pitch
    2: 2077,  # neck_pitch
    3: 1000,  # neck_yaw
    4: 2028,  # eye_yaw
    5: 3007,  # lid_left
    6: 1209,  # lid_right
}

# Default config file location — resolve symlinks first so colcon --symlink-install works.
# Walks up: maki_dxl_6.py → makimate_dxl/ → makimate_dxl/ → makimate_dxl (pkg) → core → src → repo
_REPO_ROOT = Path(__file__).resolve().parents[4]
_DEFAULT_CONFIG_PATH = str(_REPO_ROOT / "config" / "motor_limits.yaml")


class MakiDxl6(Node):
    """
    6-Dynamixel controller for MakiMate head with smooth motion interpolation.
    """

    def __init__(self):
        super().__init__('maki_dxl_6')

        # ----------------------------------------
        # ROS PARAMETERS
        # ----------------------------------------
        self.declare_parameter('port_name', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 57600)
        self.declare_parameter('ids', [1, 2, 3, 4, 5, 6])
        self.declare_parameter('smoothing_alpha', 0.18)  # Smoothing factor (0.05-0.5)
        self.declare_parameter('update_rate', 20.0)     # Hz — 57600 baud fits ~6 motors at 20Hz
        self.declare_parameter('motor_config_file', _DEFAULT_CONFIG_PATH)

        # ----------------------------------------
        # HARDWARE LIMITS — loaded from YAML or fallback to defaults
        # ----------------------------------------
        config_path = Path(self.get_parameter('motor_config_file').value)
        raw_limits, self.neutral_ticks = self._load_motor_config(config_path)

        # Convert into usable dicts
        self.min_ticks = {i: raw_limits[i]["min"] for i in raw_limits}
        self.max_ticks = {i: raw_limits[i]["max"] for i in raw_limits}

        # ----------------------------------------
        # SOFTWARE RELATIVE ANGLE LIMITS (DEG)
        # ----------------------------------------
        self.min_rel_deg = {
            1: -12.0,  # eye_pitch  (ID 1)
            2: -18.0,  # neck_pitch (ID 2)
            3: -20.0,  # neck_yaw   (ID 3, hardware ±41°, using ±20° safely)
            4: -15.0,  # eye_yaw    (ID 4)
            5: -19.0,  # lid_left   (ID 5)
            6: -26.0,  # lid_right  (ID 6)
        }
        self.max_rel_deg = {
            1: 12.0,   # eye_pitch  (ID 1)
            2: 18.0,   # neck_pitch (ID 2)
            3: 20.0,   # neck_yaw   (ID 3)
            4: 15.0,   # eye_yaw    (ID 4)
            5: 26.0,   # lid_left   (ID 5)
            6: 26.0,   # lid_right  (ID 6)
        }

        # Read basic params
        port_name = self.get_parameter('port_name').value
        baud_rate = int(self.get_parameter('baud_rate').value)
        self.ids: List[int] = [int(x) for x in self.get_parameter('ids').value]
        self.smoothing_alpha = float(self.get_parameter('smoothing_alpha').value)
        update_rate = float(self.get_parameter('update_rate').value)

        # Per-motor alpha: eyes snap instantly (1.0), lids are quick (0.5),
        # neck uses the tunable smoothing_alpha parameter.
        _EYE_IDS = {1, 4}   # eye_pitch, eye_yaw
        _LID_IDS = {5, 6}   # lid_left, lid_right
        self._smoothing_alphas = [
            1.0 if mid in _EYE_IDS else
            0.5 if mid in _LID_IDS else
            self.smoothing_alpha
            for mid in self.ids
        ]

        # ----------------------------------------
        # SMOOTHING STATE — seeded from actual motor positions after torque enable
        # ----------------------------------------
        self.current_positions = [0.0] * len(self.ids)
        self.target_positions = [0.0] * len(self.ids)
        # Ignore incoming joint goals for this many seconds after startup.
        # The expressions node publishes immediately on boot; if maki_dxl_6 is still
        # initialising (rebooting motors), the queued goals arrive the instant the
        # subscriber is active and cause a simultaneous 6-motor current spike → all
        # motors get Input Voltage / Electrical Shock errors at once.
        self._startup_time = time.monotonic()

        # ----------------------------------------
        # DYNAMIXEL SDK SETUP
        # ----------------------------------------
        self.PROTOCOL_VERSION = 2.0
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_GOAL_POSITION = 116
        self.ADDR_PRESENT_POSITION = 132
        self.ADDR_HARDWARE_ERROR_STATUS = 70  # read-only; non-zero = latched fault
        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0

        self.port_handler = PortHandler(port_name)
        self.packet_handler = PacketHandler(self.PROTOCOL_VERSION)

        if not self.port_handler.openPort():
            self.get_logger().error(f"Failed to open port {port_name}")
            raise RuntimeError("Cannot open Dynamixel port")

        if not self.port_handler.setBaudRate(baud_rate):
            self.get_logger().error(f"Failed to set baud rate {baud_rate}")
            raise RuntimeError("Cannot set baud rate")

        self.get_logger().info(
            f"Opened Dynamixel port {port_name} @ {baud_rate}bps (IDs {self.ids})"
        )

        # Enable torque — but first:
        #   1. Clear any latched hardware error by rebooting the motor.
        #      Hardware errors survive power cycles (stored in motor EEPROM) and
        #      must be cleared with a REBOOT packet before torque can be enabled.
        #   2. Write current position as goal so the motor holds still on enable
        #      instead of snapping to a stale goal register.
        #
        # NOTE: We intentionally do NOT write EEPROM position limits here.
        # Writing tight limits when the motor's physical position may be outside
        # them (e.g. after a crash or manual movement) immediately triggers a
        # position-limit hardware error the moment torque enables.  Software
        # clamping in _on_joint_goals is sufficient for normal operation.
        for dxl_id in self.ids:
            hw_err, result, _ = self.packet_handler.read1ByteTxRx(
                self.port_handler, dxl_id, self.ADDR_HARDWARE_ERROR_STATUS
            )
            if result == COMM_SUCCESS and hw_err != 0:
                self.get_logger().warn(
                    f"ID {dxl_id} has latched hardware error (0x{hw_err:02X}) — rebooting motor..."
                )
                self.packet_handler.reboot(self.port_handler, dxl_id)
                time.sleep(1.0)  # wait for motor to come back online after reboot

            present, result, _ = self.packet_handler.read4ByteTxRx(
                self.port_handler, dxl_id, self.ADDR_PRESENT_POSITION
            )
            if result == COMM_SUCCESS:
                goal_pos = present
            else:
                # Fallback: use calibrated neutral so the motor holds still rather
                # than snapping to the factory default goal (often 0 ticks), which
                # would drive it hard into its mechanical stop and trigger an error.
                goal_pos = self.neutral_ticks[dxl_id]
                self.get_logger().warn(
                    f"Could not read present position for ID {dxl_id} — "
                    f"seeding goal to neutral ({goal_pos} ticks)"
                )
            self.packet_handler.write4ByteTxRx(
                self.port_handler, dxl_id, self.ADDR_GOAL_POSITION, goal_pos
            )

            result, error = self.packet_handler.write1ByteTxRx(
                self.port_handler, dxl_id,
                self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE
            )
            if result != COMM_SUCCESS:
                self.get_logger().error(
                    f"Torque enable failed for ID {dxl_id}: "
                    f"{self.packet_handler.getTxRxResult(result)}"
                )
            elif error != 0:
                self.get_logger().error(
                    f"Dynamixel error enabling torque for ID {dxl_id}: "
                    f"{self.packet_handler.getRxPacketError(error)}"
                )
            else:
                self.get_logger().info(f"Torque enabled for ID {dxl_id}")

        # ----------------------------------------
        # SEED SMOOTHING STATE from actual present positions
        # so the first update doesn't snap to neutral.
        # ----------------------------------------
        for idx, dxl_id in enumerate(self.ids):
            ticks, result, _ = self.packet_handler.read4ByteTxRx(
                self.port_handler, dxl_id, self.ADDR_PRESENT_POSITION
            )
            if result == COMM_SUCCESS:
                rel_deg = (ticks - self.neutral_ticks[dxl_id]) / TICKS_PER_DEG
                self.current_positions[idx] = rel_deg
                self.target_positions[idx] = rel_deg
            else:
                self.get_logger().warn(
                    f"Could not read present position for ID {dxl_id} — starting from neutral"
                )

        # ----------------------------------------
        # ROS SUBSCRIBER & SMOOTH UPDATE TIMER
        # ----------------------------------------
        # Reset the startup gate here — AFTER the motor init loop completes.
        # The init loop takes ~6 s (one reboot per motor), so the gate set at the
        # top of __init__ would already be expired by the time the subscriber is
        # live.  Resetting it now means queued expression commands (published while
        # the motors were still booting) are held off for a real 3 s.
        self._startup_time = time.monotonic()

        self.sub = self.create_subscription(
            Float64MultiArray,
            '/maki/joint_goals',
            self._on_joint_goals,
            10,
        )

        # Create smooth update timer
        self.create_timer(1.0 / update_rate, self._smooth_update)
        self.add_on_set_parameters_callback(self._on_params)

        self.get_logger().info(
            f"MakiDxl6 ready with smooth motion (alpha={self.smoothing_alpha}, rate={update_rate}Hz)\n"
            "Publish [6] relative degree values to /maki/joint_goals.\n"
            "0 deg = neutral per-joint midpoint."
        )

    def _on_params(self, params):
        for p in params:
            if p.name == 'smoothing_alpha':
                self.smoothing_alpha = float(p.value)
                # Rebuild per-motor alphas (eyes/lids stay at their fixed values)
                _EYE_IDS = {1, 4}
                _LID_IDS = {5, 6}
                self._smoothing_alphas = [
                    1.0 if mid in _EYE_IDS else
                    0.5 if mid in _LID_IDS else
                    self.smoothing_alpha
                    for mid in self.ids
                ]
                self.get_logger().info(f'[live] smoothing_alpha = {self.smoothing_alpha}')
        return SetParametersResult(successful=True)

    # ----------------------------------------
    # MOTOR CONFIG LOADING
    # ----------------------------------------
    def _load_motor_config(self, path: Path):
        """Load motor limits and neutral positions from YAML. Falls back to defaults."""
        if path.exists():
            try:
                with open(path) as f:
                    cfg = yaml.safe_load(f)
                limits = {int(k): v for k, v in cfg['robot_limits'].items()}
                neutrals = {int(k): int(v) for k, v in cfg['robot_neutral_positions'].items()}
                self.get_logger().info(f"Loaded motor config from {path}")
                return limits, neutrals
            except Exception as e:
                self.get_logger().warn(
                    f"Failed to parse motor config {path}: {e} — using hardcoded defaults"
                )
        else:
            self.get_logger().warn(
                f"Motor config not found at {path} — using hardcoded defaults. "
                f"Run tools/calibrate_motors.py to generate per-robot config."
            )
        return _DEFAULT_ROBOT_LIMITS, _DEFAULT_NEUTRAL_POSITIONS

    # ----------------------------------------
    # DEGREES → TICKS CONVERSION
    # ----------------------------------------
    def _deg_to_ticks_for_id(self, dxl_id: int, angle_rel_deg: float) -> int:
        neutral = self.neutral_ticks[dxl_id]
        return int(round(neutral + angle_rel_deg * TICKS_PER_DEG))

    # ----------------------------------------
    # ON JOINT GOALS (just updates targets)
    # ----------------------------------------
    def _on_joint_goals(self, msg: Float64MultiArray):
        # Discard commands that arrive in the first 3 s after startup.
        # The expressions node queues commands before the motors are ready;
        # executing a large multi-motor movement immediately causes a current
        # spike that triggers Input Voltage / Electrical Shock errors on all
        # motors at once.
        if time.monotonic() - self._startup_time < 3.0:
            return

        values = list(msg.data)
        if len(values) != len(self.ids):
            self.get_logger().warning(
                f"Expected {len(self.ids)} joints but got {len(values)}"
            )
            return

        # Update target positions with clamping
        for idx, (dxl_id, angle_rel) in enumerate(zip(self.ids, values)):
            # Clamp to software limits
            min_d = self.min_rel_deg.get(dxl_id, -30.0)
            max_d = self.max_rel_deg.get(dxl_id, 30.0)
            clamped = max(min_d, min(max_d, angle_rel))
            
            self.target_positions[idx] = clamped

    # ----------------------------------------
    # SMOOTH UPDATE LOOP
    # ----------------------------------------
    def _smooth_update(self):
        """Smoothly interpolate toward target positions and send to servos."""
        for idx, dxl_id in enumerate(self.ids):
            delta = self.target_positions[idx] - self.current_positions[idx]
            # Snap when very close to target — stops the servo from hunting at
            # steady state (endless micro-corrections cause visible wobble)
            if abs(delta) < 0.15:
                self.current_positions[idx] = self.target_positions[idx]
            else:
                self.current_positions[idx] += self._smoothing_alphas[idx] * delta

            # Convert to ticks
            ticks = self._deg_to_ticks_for_id(dxl_id, self.current_positions[idx])

            # Clamp to hardware limits (avoid boundary values)
            ticks = max(
                self.min_ticks[dxl_id] + 1,
                min(self.max_ticks[dxl_id] - 1, ticks)
            )

            # Send to servo — catch serial errors so a transient glitch doesn't crash the node
            try:
                result, error = self.packet_handler.write4ByteTxRx(
                    self.port_handler, dxl_id,
                    self.ADDR_GOAL_POSITION, ticks
                )
            except Exception as e:
                self.get_logger().warn(
                    f'Serial error on ID {dxl_id}: {e}', throttle_duration_sec=2.0
                )
                continue

            if result != COMM_SUCCESS:
                self.get_logger().warn(
                    f"Failed to set ID {dxl_id} goal: "
                    f"{self.packet_handler.getTxRxResult(result)}",
                    throttle_duration_sec=2.0
                )
            elif error != 0:
                self.get_logger().warn(
                    f"Dynamixel error on ID {dxl_id}: "
                    f"{self.packet_handler.getRxPacketError(error)}",
                    throttle_duration_sec=2.0
                )

    # ----------------------------------------
    # CLEAN SHUTDOWN
    # ----------------------------------------
    def destroy_node(self):
        self.get_logger().info("Disabling torque + closing port...")
        for dxl_id in self.ids:
            try:
                self.packet_handler.write1ByteTxRx(
                    self.port_handler, dxl_id,
                    self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE
                )
            except Exception:
                pass
        try:
            self.port_handler.closePort()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MakiDxl6()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
