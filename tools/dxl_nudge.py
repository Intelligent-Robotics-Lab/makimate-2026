#!/usr/bin/env python3
"""
Dynamixel per-motor nudge tester.

Lets you pick a motor ID and nudge it +/- ticks to confirm which physical
joint it controls.  Safe to run while the service is stopped.

Usage:
    sudo systemctl stop makimate
    python3 ~/makimate-2026/tools/dxl_nudge.py

Controls:
    Enter an ID (1-6) to select a motor, then:
        +  nudge forward (higher ticks)
        -  nudge backward (lower ticks)
        0  return motor to current present position (hold still)
        q  quit / select a different motor
        x  exit
"""

import sys
import tty
import termios
import time
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

PORT     = "/dev/ttyACM0"
BAUD     = 57600
PROTOCOL = 2.0
IDS      = [1, 2, 3, 4, 5, 6]
STEP     = 50   # ticks per nudge

JOINT_NAMES = {
    1: "neck_yaw",
    2: "neck_pitch",
    3: "eye_pitch",
    4: "eye_yaw",
    5: "lid_left",
    6: "lid_right",
}

ADDR_TORQUE_ENABLE    = 64
ADDR_GOAL_POSITION    = 116
ADDR_PRESENT_POSITION = 132
ADDR_MAX_POS_LIMIT    = 48
ADDR_MIN_POS_LIMIT    = 52


def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    port   = PortHandler(PORT)
    packet = PacketHandler(PROTOCOL)

    if not port.openPort():
        print(f"ERROR: Cannot open {PORT}. Is the service stopped?")
        return
    if not port.setBaudRate(BAUD):
        print("ERROR: Cannot set baud rate.")
        return

    print(f"Opened {PORT} @ {BAUD} bps")
    print()

    # Ensure full EEPROM position range on all motors so nudges aren't blocked
    for dxl_id in IDS:
        packet.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 0)
        packet.write4ByteTxRx(port, dxl_id, ADDR_MAX_POS_LIMIT, 4095)
        packet.write4ByteTxRx(port, dxl_id, ADDR_MIN_POS_LIMIT, 0)

    try:
        while True:
            print("Select motor ID to test [1-6, or x to exit]:", end=" ")
            choice = input().strip().lower()
            if choice == 'x':
                break
            try:
                dxl_id = int(choice)
                if dxl_id not in IDS:
                    raise ValueError
            except ValueError:
                print("  Invalid choice.")
                continue

            name = JOINT_NAMES[dxl_id]
            print(f"\nTesting ID {dxl_id} ({name})")
            print(f"  +/- nudge ({STEP} ticks)  |  0 = hold present  |  q = back to menu")

            # Enable torque on this motor only
            packet.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 1)

            # Read current position as starting goal
            pos, result, _ = packet.read4ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)
            if result != COMM_SUCCESS or pos > 4095:
                pos = 2048
                print(f"  Could not read position — starting from 2048")
            goal = pos
            packet.write4ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, goal)
            print(f"  Current position: {pos}")

            while True:
                key = getch()
                if key == 'q' or key == '\x03':
                    break
                elif key == 'x':
                    packet.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 0)
                    port.closePort()
                    return
                elif key == '+' or key == '=':
                    goal = min(4095, goal + STEP)
                elif key == '-':
                    goal = max(0, goal - STEP)
                elif key == '0':
                    p, r, _ = packet.read4ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)
                    if r == COMM_SUCCESS and p <= 4095:
                        goal = p
                else:
                    continue

                result, error = packet.write4ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, goal)
                time.sleep(0.05)
                p2, r2, _ = packet.read4ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)
                actual = p2 if r2 == COMM_SUCCESS else "?"
                sys.stdout.write(f"\r  goal={goal:4d}  present={str(actual):4s}    ")
                sys.stdout.flush()

            print()
            # Disable torque when leaving this motor
            packet.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 0)
            print(f"  Torque disabled for ID {dxl_id}.")

    finally:
        for dxl_id in IDS:
            packet.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 0)
        port.closePort()
        print("\nDone.")


if __name__ == "__main__":
    main()
