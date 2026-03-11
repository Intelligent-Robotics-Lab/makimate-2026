#!/usr/bin/env python3
import math
from typing import List

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
# ROBOT-SPECIFIC LIMITS (EDIT HERE FOR DIFFERENT ROBOTS)
# ----------------------------------------------------
ROBOT_LIMITS = {
    1: {"min": 28, "max": 570},
    2: {"min": 1748, "max": 2348},
    3: {"min": 532, "max": 1467},
    4: {"min": 1809, "max": 2241},
    5: {"min": 2432, "max": 3121},
    6: {"min": 1097, "max": 1786},
}

# ACTUAL NEUTRAL POSITIONS (measured, not calculated!)
ROBOT_NEUTRAL_POSITIONS = {
    1: 460,   # neck yaw
    2: 2077,  # neck pitch
    3: 1000,  # eyes pitch
    4: 2028,  # eyes yaw
    5: 3007,  # lid left
    6: 1209,  # lid right
}


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
        self.declare_parameter('smoothing_alpha', 0.2)  # Smoothing factor (0.05-0.5)
        self.declare_parameter('update_rate', 50.0)     # Hz

        # ----------------------------------------
        # HARDWARE LIMITS FROM ROBOT_LIMITS
        # ----------------------------------------
        raw_limits = ROBOT_LIMITS

        # Convert into usable dicts
        self.min_ticks = {i: raw_limits[i]["min"] for i in raw_limits}
        self.max_ticks = {i: raw_limits[i]["max"] for i in raw_limits}

        # Use MEASURED neutral positions instead of calculated midpoint
        self.neutral_ticks = ROBOT_NEUTRAL_POSITIONS

        # ----------------------------------------
        # SOFTWARE RELATIVE ANGLE LIMITS (DEG)
        # ----------------------------------------
        self.min_rel_deg = {
            1: -9.0,   # neck_yaw
            2: -18.0,  # neck_pitch
            3: -12.0,  # eyes_pitch
            4: -15.0,  # eyes_yaw
            5: -19.0,  # lid_left
            6: -26.0,  # lid_right
        }
        self.max_rel_deg = {
            1: 9.0,    # neck_yaw
            2: 18.0,   # neck_pitch
            3: 10.0,   # eyes_pitch
            4: 15.0,   # eyes_yaw
            5: 26.0,   # lid_left
            6: 26.0,   # lid_right
        }

        # Read basic params
        port_name = self.get_parameter('port_name').value
        baud_rate = int(self.get_parameter('baud_rate').value)
        self.ids: List[int] = [int(x) for x in self.get_parameter('ids').value]
        self.smoothing_alpha = float(self.get_parameter('smoothing_alpha').value)
        update_rate = float(self.get_parameter('update_rate').value)

        # ----------------------------------------
        # SMOOTHING STATE
        # ----------------------------------------
        self.current_positions = [0.0] * len(self.ids)  # Current smooth positions (degrees)
        self.target_positions = [0.0] * len(self.ids)   # Target positions (degrees)

        # ----------------------------------------
        # DYNAMIXEL SDK SETUP
        # ----------------------------------------
        self.PROTOCOL_VERSION = 2.0
        self.ADDR_TORQUE_ENABLE = 64
        self.ADDR_GOAL_POSITION = 116
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

            # Send to servo
            result, error = self.packet_handler.write4ByteTxRx(
                self.port_handler, dxl_id,
                self.ADDR_GOAL_POSITION, ticks
            )

            if result != COMM_SUCCESS:
                self.get_logger().error(
                    f"Failed to set ID {dxl_id} goal: "
                    f"{self.packet_handler.getTxRxResult(result)}"
                )
            elif error != 0:
                self.get_logger().error(
                    f"Dynamixel error on ID {dxl_id}: "
                    f"{self.packet_handler.getRxPacketError(error)}"
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
