#!/usr/bin/env python3
"""
MakiMate Motor Calibration Tool
================================
Interactive tool to find the hardware min / max / neutral tick positions for
each of the 6 Dynamixel motors and save them to config/motor_limits.yaml.

Run this ONCE on each new robot before launching MakiMate.

Usage:
    python3 tools/calibrate_motors.py [--port /dev/ttyACM0] [--baud 57600]
    python3 tools/calibrate_motors.py --output /path/to/motor_limits.yaml

Controls (while positioning a motor):
    +  or  →   nudge motor toward higher tick value
    -  or  ←   nudge motor toward lower tick value
    [          halve the step size
    ]          double the step size
    m          mark current position as MIN limit
    x          mark current position as MAX limit
    n          mark current position as NEUTRAL (home / rest position)
    s          skip this motor (keep any existing YAML values, or defaults)
    Enter      accept all three marks and continue to next motor

IMPORTANT: move slowly and gently near the physical stops.
Do NOT force a motor past its mechanical limit.
"""

import argparse
import sys
import tty
import termios
import time
from pathlib import Path
from typing import Dict, Optional, Tuple

try:
    from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
except ImportError:
    print("ERROR: dynamixel_sdk not found. Install with: pip install dynamixel-sdk")
    sys.exit(1)

try:
    import yaml
except ImportError:
    print("ERROR: PyYAML not found. Install with: pip install PyYAML")
    sys.exit(1)

# -------------------------------------------------------------------
# Dynamixel Protocol 2.0 register addresses (MX-series / XM-series)
# -------------------------------------------------------------------
PROTOCOL_VERSION    = 2.0
ADDR_TORQUE_ENABLE  = 64   # 1 byte
ADDR_GOAL_POSITION  = 116  # 4 bytes
ADDR_PRESENT_POS    = 132  # 4 bytes (read-only)
TORQUE_ENABLE       = 1
TORQUE_DISABLE      = 0

MOTOR_NAMES = {
    1: "neck_yaw",
    2: "neck_pitch",
    3: "eye_pitch",
    4: "eye_yaw",
    5: "lid_left",
    6: "lid_right",
}

# Default values shown when no prior YAML exists
_FALLBACK_MIN  = {1: 2640, 2: 1855, 3: 2352, 4: 1679, 5: 2378, 6: 1021}
_FALLBACK_MAX  = {1: 3641, 2: 2324, 3: 2635, 4: 2495, 5: 3057, 6: 1699}
_FALLBACK_NEU  = {1: 3140, 2: 2090, 3: 2493, 4: 2087, 5: 2717, 6: 1360}

REPO_ROOT    = Path(__file__).parent.parent
DEFAULT_OUT  = REPO_ROOT / "config" / "motor_limits.yaml"


# -------------------------------------------------------------------
# Terminal helpers
# -------------------------------------------------------------------

def _getch() -> str:
    """Read a single keypress without Enter (Unix only)."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        # Handle escape sequences (arrow keys = ESC [ A/B/C/D)
        if ch == '\x1b':
            ch2 = sys.stdin.read(1)
            if ch2 == '[':
                ch3 = sys.stdin.read(1)
                return {'A': 'UP', 'B': 'DOWN', 'C': 'RIGHT', 'D': 'LEFT'}.get(ch3, '')
            return ''
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def _clear_line():
    sys.stdout.write('\r\033[K')
    sys.stdout.flush()


def _status_line(dxl_id, name, pos, step, marks):
    min_s = str(marks.get('min', '---'))
    max_s = str(marks.get('max', '---'))
    neu_s = str(marks.get('neutral', '---'))
    line = (
        f"  ID {dxl_id} [{name:10s}]  pos={pos:5d}  step={step:4d}"
        f"   min={min_s:5s}  max={max_s:5s}  neu={neu_s:5s}"
        f"   [m]in [x]max [n]eutral  +/- nudge  [s]kip  Enter=confirm"
    )
    _clear_line()
    sys.stdout.write(line)
    sys.stdout.flush()


# -------------------------------------------------------------------
# Dynamixel helpers
# -------------------------------------------------------------------

def read_position(ph: PacketHandler, port, dxl_id: int) -> Optional[int]:
    val, result, error = ph.read4ByteTxRx(port, dxl_id, ADDR_PRESENT_POS)
    if result != COMM_SUCCESS or error != 0:
        return None
    return val


def write_position(ph: PacketHandler, port, dxl_id: int, pos: int) -> bool:
    result, error = ph.write4ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, pos)
    return result == COMM_SUCCESS and error == 0


def set_torque(ph: PacketHandler, port, dxl_id: int, enable: bool):
    ph.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE,
                      TORQUE_ENABLE if enable else TORQUE_DISABLE)


# -------------------------------------------------------------------
# Load existing YAML (if any)
# -------------------------------------------------------------------

def load_existing(path: Path) -> Tuple[Dict, Dict, Dict]:
    if path.exists():
        try:
            with open(path) as f:
                cfg = yaml.safe_load(f)
            limits   = {int(k): v for k, v in cfg.get('robot_limits', {}).items()}
            neutrals = {int(k): int(v) for k, v in cfg.get('robot_neutral_positions', {}).items()}
            mins = {mid: limits[mid]['min'] for mid in limits}
            maxs = {mid: limits[mid]['max'] for mid in limits}
            print(f"Loaded existing config from {path}")
            return mins, maxs, neutrals
        except Exception as e:
            print(f"Warning: could not parse existing config ({e}), starting fresh.")
    return dict(_FALLBACK_MIN), dict(_FALLBACK_MAX), dict(_FALLBACK_NEU)


# -------------------------------------------------------------------
# Save YAML
# -------------------------------------------------------------------

def save_yaml(path: Path, mins: Dict, maxs: Dict, neutrals: Dict):
    path.parent.mkdir(parents=True, exist_ok=True)
    cfg = {
        'robot_limits': {
            mid: {'min': mins[mid], 'max': maxs[mid]}
            for mid in sorted(mins)
        },
        'robot_neutral_positions': {
            mid: neutrals[mid]
            for mid in sorted(neutrals)
        },
    }
    # Add friendly comments by writing manually
    lines = [
        "# MakiMate per-robot motor configuration",
        "# Generated by: tools/calibrate_motors.py",
        "#",
        "# Motor IDs:",
        "#   1 = eye_pitch",
        "#   2 = neck_pitch",
        "#   3 = neck_yaw",
        "#   4 = eye_yaw",
        "#   5 = lid_left",
        "#   6 = lid_right",
        "",
        "robot_limits:",
    ]
    for mid in sorted(mins):
        name = MOTOR_NAMES.get(mid, f"id{mid}")
        lines.append(f"  {mid}: {{min: {mins[mid]:5d}, max: {maxs[mid]:5d}}}    # {name}")
    lines.append("")
    lines.append("robot_neutral_positions:")
    for mid in sorted(neutrals):
        name = MOTOR_NAMES.get(mid, f"id{mid}")
        lines.append(f"  {mid}: {neutrals[mid]:5d}    # {name}")
    lines.append("")

    with open(path, 'w') as f:
        f.write('\n'.join(lines))
    print(f"\nSaved to {path}")


# -------------------------------------------------------------------
# Calibrate one motor interactively
# -------------------------------------------------------------------

def calibrate_motor(
    ph: PacketHandler, port, dxl_id: int,
    cur_min: int, cur_max: int, cur_neutral: int
) -> Optional[Tuple[int, int, int]]:
    """
    Returns (min, max, neutral) or None if skipped.
    """
    name = MOTOR_NAMES.get(dxl_id, f"id{dxl_id}")

    print(f"\n{'='*70}")
    print(f"  Calibrating motor ID {dxl_id}: {name}")
    print(f"  Current values  min={cur_min}  max={cur_max}  neutral={cur_neutral}")
    print(f"  Controls: +/→ nudge up  -/← nudge down  [ ] adjust step")
    print(f"            [m] mark min  [x] mark max  [n] mark neutral")
    print(f"            [s] skip this motor  Enter = confirm and continue")
    print(f"{'='*70}")

    pos = read_position(ph, port, dxl_id)
    if pos is None:
        print(f"  ERROR: Cannot read position for ID {dxl_id}. Motor offline?")
        return None

    step   = 10
    marks  = {'min': cur_min, 'max': cur_max, 'neutral': cur_neutral}

    while True:
        _status_line(dxl_id, name, pos, step, marks)
        key = _getch()

        if key in ('+', 'RIGHT'):
            new_pos = pos + step
            if write_position(ph, port, dxl_id, new_pos):
                time.sleep(0.15)
                p = read_position(ph, port, dxl_id)
                pos = p if p is not None else pos

        elif key in ('-', 'LEFT'):
            new_pos = max(0, pos - step)
            if write_position(ph, port, dxl_id, new_pos):
                time.sleep(0.15)
                p = read_position(ph, port, dxl_id)
                pos = p if p is not None else pos

        elif key == '[':
            step = max(1, step // 2)

        elif key == ']':
            step = min(500, step * 2)

        elif key == 'm':
            marks['min'] = pos
            sys.stdout.write(f'  → MIN marked at {pos}')
            sys.stdout.flush()

        elif key == 'x':
            marks['max'] = pos
            sys.stdout.write(f'  → MAX marked at {pos}')
            sys.stdout.flush()

        elif key == 'n':
            marks['neutral'] = pos
            sys.stdout.write(f'  → NEUTRAL marked at {pos}')
            sys.stdout.flush()

        elif key == 's':
            print(f"\n  Skipped ID {dxl_id} — keeping existing values.")
            return None

        elif key in ('\r', '\n'):
            # Validate
            if marks['min'] >= marks['max']:
                print("\n  ERROR: min must be less than max. Keep adjusting.")
                continue
            if not (marks['min'] <= marks['neutral'] <= marks['max']):
                print("\n  WARNING: neutral is outside [min, max]. Proceed anyway? (y/n)")
                confirm = _getch()
                if confirm.lower() != 'y':
                    continue
            print(f"\n  Confirmed: min={marks['min']}  max={marks['max']}  neutral={marks['neutral']}")
            return marks['min'], marks['max'], marks['neutral']

        elif key == '\x03':  # Ctrl-C
            print("\nAborted.")
            sys.exit(0)


# -------------------------------------------------------------------
# Main
# -------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="MakiMate motor calibration tool")
    parser.add_argument('--port',   default='/dev/ttyACM0', help='Serial port (default: /dev/ttyACM0)')
    parser.add_argument('--baud',   default=57600, type=int, help='Baud rate (default: 57600)')
    parser.add_argument('--ids',    default='1,2,3,4,5,6',  help='Motor IDs to calibrate (default: 1,2,3,4,5,6)')
    parser.add_argument('--output', default=str(DEFAULT_OUT), help=f'Output YAML (default: {DEFAULT_OUT})')
    args = parser.parse_args()

    motor_ids = [int(x) for x in args.ids.split(',')]
    output    = Path(args.output)

    # Open port
    port = PortHandler(args.port)
    ph   = PacketHandler(PROTOCOL_VERSION)

    if not port.openPort():
        print(f"ERROR: Cannot open port {args.port}")
        sys.exit(1)
    if not port.setBaudRate(args.baud):
        print(f"ERROR: Cannot set baud rate {args.baud}")
        sys.exit(1)

    print(f"Opened {args.port} @ {args.baud} baud")
    print(f"Output will be saved to: {output}")

    # Load existing config as starting values
    mins, maxs, neutrals = load_existing(output)

    # Enable torque on all target motors
    for mid in motor_ids:
        set_torque(ph, port, mid, True)

    try:
        for mid in motor_ids:
            result = calibrate_motor(
                ph, port, mid,
                mins.get(mid, _FALLBACK_MIN.get(mid, 100)),
                maxs.get(mid, _FALLBACK_MAX.get(mid, 3900)),
                neutrals.get(mid, _FALLBACK_NEU.get(mid, 2048)),
            )
            if result is not None:
                mins[mid], maxs[mid], neutrals[mid] = result

        # Move all motors to neutral before finishing
        print("\nMoving all motors to neutral positions...")
        for mid in motor_ids:
            write_position(ph, port, mid, neutrals[mid])
        time.sleep(1.5)

    finally:
        for mid in motor_ids:
            set_torque(ph, port, mid, False)
        port.closePort()

    save_yaml(output, mins, maxs, neutrals)
    print("\nCalibration complete. Rebuild/restart maki_dxl_6 to apply changes.")


if __name__ == '__main__':
    main()
