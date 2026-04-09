#!/usr/bin/env python3
"""
Dynamixel hard-reset tool.

Resets all 6 motors to a known-good state:
  1. Reads and reports current Operating Mode + position
  2. If Operating Mode != 3 (Position Control), rewrites it to 3
  3. Reboots every motor (clears all latched hardware errors)
  4. After reboot, reads present position
  5. If position is outside valid 0-4095 range, uses the calibrated neutral
  6. Writes that position as the goal so the motor holds still on torque enable
  7. Enables torque

Run with the service stopped:
    sudo systemctl stop makimate
    python3 ~/makimate-2026/tools/dxl_reset.py

After this script completes the motors should be in a clean state.
You can then start the service again:
    sudo systemctl start makimate
"""

import time
import yaml
from pathlib import Path
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

PORT     = "/dev/ttyACM0"
BAUD     = 57600
PROTOCOL = 2.0
IDS      = [1, 2, 3, 4, 5, 6]

ADDR_OPERATING_MODE        = 11   # 1 byte, EEPROM — write with torque OFF
ADDR_MAX_POSITION_LIMIT    = 48   # 4 bytes, EEPROM — write with torque OFF
ADDR_MIN_POSITION_LIMIT    = 52   # 4 bytes, EEPROM — write with torque OFF
ADDR_TORQUE_ENABLE         = 64   # 1 byte
ADDR_GOAL_POSITION         = 116  # 4 bytes
ADDR_PRESENT_POSITION      = 132  # 4 bytes
ADDR_HARDWARE_ERROR_STATUS = 70   # 1 byte (read-only)

OPERATING_MODE_POSITION = 3   # Normal 0–4095 single-turn position control

JOINT_NAMES = {1: "neck_yaw",  2: "neck_pitch", 3: "eye_pitch",
               4: "eye_yaw",  5: "lid_left",   6: "lid_right"}

# Load calibrated neutral positions from motor_limits.yaml if available
_REPO_ROOT = Path(__file__).resolve().parents[1]
_YAML_PATH = _REPO_ROOT / "config" / "motor_limits.yaml"

NEUTRAL = {1: 460, 2: 2077, 3: 1000, 4: 2028, 5: 3007, 6: 1209}  # defaults
if _YAML_PATH.exists():
    try:
        cfg = yaml.safe_load(_YAML_PATH.read_text())
        NEUTRAL = {int(k): int(v) for k, v in cfg["robot_neutral_positions"].items()}
        print(f"Loaded neutrals from {_YAML_PATH}")
    except Exception as e:
        print(f"Warning: could not load YAML ({e}), using hardcoded neutrals")


def main():
    port   = PortHandler(PORT)
    packet = PacketHandler(PROTOCOL)

    if not port.openPort():
        print(f"ERROR: Failed to open {PORT}. Is the service stopped?")
        print("       sudo systemctl stop makimate")
        return
    if not port.setBaudRate(BAUD):
        print(f"ERROR: Failed to set baud {BAUD}")
        return

    print(f"\nOpened {PORT} @ {BAUD} bps\n{'='*55}")

    # ── Step 1: Report current state ──────────────────────────────────────────
    print("Current state before reset:")
    for dxl_id in IDS:
        mode, comm, _ = packet.read1ByteTxRx(port, dxl_id, ADDR_OPERATING_MODE)
        pos,  comm2,_ = packet.read4ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)
        hw,   comm3,_ = packet.read1ByteTxRx(port, dxl_id, ADDR_HARDWARE_ERROR_STATUS)
        mode_str = str(mode) if comm  == COMM_SUCCESS else "ERR"
        pos_str  = str(pos)  if comm2 == COMM_SUCCESS else "ERR"
        hw_str   = f"0x{hw:02X}" if comm3 == COMM_SUCCESS else "ERR"
        print(f"  ID {dxl_id} ({JOINT_NAMES[dxl_id]:10s})  mode={mode_str}  pos={pos_str:12s}  hw_err={hw_str}")

    print()

    # ── Step 2: Fix operating mode if needed (torque must be OFF first) ───────
    print("Checking/fixing Operating Mode (must be 3 = Position Control)...")
    for dxl_id in IDS:
        # Ensure torque is off before touching EEPROM
        packet.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 0)

        mode, comm, _ = packet.read1ByteTxRx(port, dxl_id, ADDR_OPERATING_MODE)
        if comm != COMM_SUCCESS:
            print(f"  ID {dxl_id}: can't read operating mode (comm error)")
            continue

        if mode != OPERATING_MODE_POSITION:
            print(f"  ID {dxl_id}: Operating Mode = {mode} — resetting to 3 (Position Control)...")
            result, _ = packet.write1ByteTxRx(
                port, dxl_id, ADDR_OPERATING_MODE, OPERATING_MODE_POSITION
            )
            if result == COMM_SUCCESS:
                print(f"  ID {dxl_id}: Operating Mode reset OK")
            else:
                print(f"  ID {dxl_id}: ERROR writing operating mode: {packet.getTxRxResult(result)}")
        else:
            print(f"  ID {dxl_id}: Operating Mode already 3 — OK")

    print()

    # ── Step 3: Reboot all motors (clears latched hardware errors) ────────────
    print("Rebooting all motors to clear latched hardware errors...")
    for dxl_id in IDS:
        result, err = packet.reboot(port, dxl_id)
        if result == COMM_SUCCESS and err == 0:
            print(f"  ID {dxl_id}: reboot sent")
        else:
            print(f"  ID {dxl_id}: reboot error — {packet.getTxRxResult(result)}")

    print("Waiting 2 s for motors to come back online...")
    time.sleep(2.0)
    print()

    # ── Step 3b: Clear EEPROM position limits → full 0–4095 range ─────────────
    # Previous wrong calibration may have written tight limits (e.g. max=570 for
    # ID 1) into EEPROM.  These survive reboots and cause "data value exceeds
    # limit" errors when we write a goal outside that stale range.
    # Torque is off after reboot — safe to write EEPROM now.
    print("Clearing EEPROM position limits to full range (0–4095)...")
    for dxl_id in IDS:
        packet.write4ByteTxRx(port, dxl_id, ADDR_MAX_POSITION_LIMIT, 4095)
        packet.write4ByteTxRx(port, dxl_id, ADDR_MIN_POSITION_LIMIT, 0)
        print(f"  ID {dxl_id}: position limits cleared")
    print()

    # ── Step 4: Set goal = present (or neutral if position is garbage) ─────────
    print("Setting goal positions and enabling torque...")
    for dxl_id in IDS:
        pos, comm, _ = packet.read4ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)

        if comm != COMM_SUCCESS or pos > 4095:
            goal = NEUTRAL[dxl_id]
            reason = f"position read failed or out of range ({pos})" if comm == COMM_SUCCESS else "comm error"
            print(f"  ID {dxl_id}: {reason} — using neutral ({goal})")
        else:
            goal = pos
            print(f"  ID {dxl_id}: present position = {pos} — using as goal")

        packet.write4ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, goal)

        result, error = packet.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 1)
        if result == COMM_SUCCESS and error == 0:
            print(f"  ID {dxl_id}: torque enabled ✓")
        else:
            print(f"  ID {dxl_id}: torque enable failed — {packet.getTxRxResult(result)}")

    print()

    # ── Step 5: Final state report ────────────────────────────────────────────
    print("Final state after reset:")
    for dxl_id in IDS:
        mode, comm, _ = packet.read1ByteTxRx(port, dxl_id, ADDR_OPERATING_MODE)
        pos,  comm2,_ = packet.read4ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)
        hw,   comm3,_ = packet.read1ByteTxRx(port, dxl_id, ADDR_HARDWARE_ERROR_STATUS)
        torq, comm4,_ = packet.read1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE)
        mode_str = str(mode) if comm  == COMM_SUCCESS else "ERR"
        pos_str  = str(pos)  if comm2 == COMM_SUCCESS else "ERR"
        hw_str   = f"0x{hw:02X}" if comm3 == COMM_SUCCESS else "ERR"
        torq_str = ("ON" if torq else "off") if comm4 == COMM_SUCCESS else "ERR"
        print(f"  ID {dxl_id} ({JOINT_NAMES[dxl_id]:10s})  mode={mode_str}  pos={pos_str:6s}  hw_err={hw_str}  torque={torq_str}")

    print(f"\n{'='*55}")
    print("Reset complete.")
    print("You can now start the service:  sudo systemctl start makimate")
    port.closePort()


if __name__ == "__main__":
    main()
