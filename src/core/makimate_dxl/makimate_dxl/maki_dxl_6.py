#!/usr/bin/env python3
import math
from pathlib import Path
from typing import List

import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

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

        # ----------------------------------------
        # SMOOTHING STATE — seeded from actual motor positions after torque enable
        # ----------------------------------------
        self.current_positions = [0.0] * len(self.ids)
        self.target_positions = [0.0] * len(self.ids)

        # ----------------------------------------
        # DYNAMIXEL SDK SETUP
        # ----------------------------------------
        self.PROTOCOL_VERSION = 2.0
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_GOAL_POSITION = 116
        self.ADDR_PRESENT_POSITION = 132
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

        # Enable torque on all motors
        for dxl_id in self.ids:
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
        self.sub = self.create_subscription(
            Float64MultiArray,
            '/maki/joint_goals',
            self._on_joint_goals,
            10,
        )

        # Create smooth update timer
        self.create_timer(1.0 / update_rate, self._smooth_update)

        self.get_logger().info(
            f"MakiDxl6 ready with smooth motion (alpha={self.smoothing_alpha}, rate={update_rate}Hz)\n"
            "Publish [6] relative degree values to /maki/joint_goals.\n"
            "0 deg = neutral per-joint midpoint."
        )

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
            # Exponential smoothing: gradually move current toward target
            self.current_positions[idx] += self.smoothing_alpha * (
                self.target_positions[idx] - self.current_positions[idx]
            )

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
