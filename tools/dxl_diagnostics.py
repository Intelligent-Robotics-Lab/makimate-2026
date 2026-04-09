#!/usr/bin/env python3
"""
Dynamixel diagnostic tool — reads voltage and error state for all 6 motors.

Usage (stop the service first!):
    sudo systemctl stop makimate
    python3 ~/makimate-2026/tools/dxl_diagnostics.py

Output includes:
  - Present Input Voltage (what the motor sees on the 12V bus)
  - Min/Max Voltage Limits (thresholds stored in motor EEPROM)
  - Hardware Error Status register (latched fault bits)
  - Present Position (current tick value)
  - Shutdown register (which errors cause automatic torque disable)

Error bit meanings (Hardware Error Status):
  Bit 0 (0x01): Input Voltage Error  — supply below Min Voltage Limit
  Bit 2 (0x04): Overheating Error
  Bit 3 (0x08): Motor Encoder Error
  Bit 4 (0x10): Electrical Shock Error — abnormal current spike
  Bit 5 (0x20): Overload Error
"""

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

PORT     = "/dev/ttyACM0"
BAUD     = 57600
PROTOCOL = 2.0
IDS      = [1, 2, 3, 4, 5, 6]

ADDR_MIN_VOLT_LIMIT        = 34   # 1 byte, units: 0.1 V
ADDR_MAX_VOLT_LIMIT        = 32   # 1 byte, units: 0.1 V
ADDR_SHUTDOWN              = 63   # 1 byte, bitmask of errors that disable torque
ADDR_TORQUE_ENABLE         = 64   # 1 byte
ADDR_HARDWARE_ERROR_STATUS = 70   # 1 byte (read-only)
ADDR_PRESENT_INPUT_VOLTAGE = 144  # 2 bytes, units: 0.1 V
ADDR_PRESENT_POSITION      = 132  # 4 bytes

JOINT_NAMES = {
    1: "neck_yaw",
    2: "neck_pitch",
    3: "eye_pitch",
    4: "eye_yaw",
    5: "lid_left",
    6: "lid_right",
}

SHUTDOWN_BITS = {
    0: "Input Voltage",
    2: "Overheating",
    3: "Motor Encoder",
    4: "Electrical Shock",
    5: "Overload",
}

ERROR_BITS = {
    0: "Input Voltage",
    2: "Overheating",
    3: "Motor Encoder",
    4: "Electrical Shock",
    5: "Overload",
}


def decode_bits(value, bit_map):
    active = [label for bit, label in sorted(bit_map.items()) if value & (1 << bit)]
    return ", ".join(active) if active else "none"


def read1(packet, port, dxl_id, addr, label):
    val, comm, err = packet.read1ByteTxRx(port, dxl_id, addr)
    if comm != COMM_SUCCESS:
        return None, f"  {label}: COMM ERROR ({packet.getTxRxResult(comm)})"
    return val, None


def read2(packet, port, dxl_id, addr, label):
    val, comm, err = packet.read2ByteTxRx(port, dxl_id, addr)
    if comm != COMM_SUCCESS:
        return None, f"  {label}: COMM ERROR ({packet.getTxRxResult(comm)})"
    return val, None


def read4(packet, port, dxl_id, addr, label):
    val, comm, err = packet.read4ByteTxRx(port, dxl_id, addr)
    if comm != COMM_SUCCESS:
        return None, f"  {label}: COMM ERROR ({packet.getTxRxResult(comm)})"
    return val, None


def main():
    port   = PortHandler(PORT)
    packet = PacketHandler(PROTOCOL)

    if not port.openPort():
        print(f"ERROR: Failed to open port {PORT}")
        print("Is the service stopped?  sudo systemctl stop makimate")
        return
    if not port.setBaudRate(BAUD):
        print(f"ERROR: Failed to set baud rate {BAUD}")
        return

    print(f"Opened {PORT} @ {BAUD} bps\n")
    print("=" * 55)

    for dxl_id in IDS:
        name = JOINT_NAMES.get(dxl_id, "?")
        print(f"ID {dxl_id}  ({name})")

        # Voltage
        vin, err = read2(packet, port, dxl_id, ADDR_PRESENT_INPUT_VOLTAGE, "Voltage")
        if err:
            print(err)
        else:
            print(f"  Present Input Voltage : {vin/10:.1f} V  (raw {vin})")

        min_v, err = read1(packet, port, dxl_id, ADDR_MIN_VOLT_LIMIT, "Min Volt Limit")
        max_v, err2 = read1(packet, port, dxl_id, ADDR_MAX_VOLT_LIMIT, "Max Volt Limit")
        if min_v is not None and max_v is not None:
            print(f"  Voltage Limits (EEPROM): {min_v/10:.1f} V – {max_v/10:.1f} V")

        # Hardware error
        hw, err = read1(packet, port, dxl_id, ADDR_HARDWARE_ERROR_STATUS, "HW Error")
        if err:
            print(err)
        else:
            flags = decode_bits(hw, ERROR_BITS)
            marker = "  *** LATCHED ERROR ***" if hw != 0 else ""
            print(f"  Hardware Error Status : 0x{hw:02X}  [{flags}]{marker}")

        # Shutdown register
        sd, err = read1(packet, port, dxl_id, ADDR_SHUTDOWN, "Shutdown")
        if sd is not None:
            flags = decode_bits(sd, SHUTDOWN_BITS)
            print(f"  Shutdown Register     : 0x{sd:02X}  [{flags}]")

        # Present position
        pos, err = read4(packet, port, dxl_id, ADDR_PRESENT_POSITION, "Position")
        if pos is not None:
            print(f"  Present Position      : {pos} ticks")

        # Torque state
        torque, err = read1(packet, port, dxl_id, ADDR_TORQUE_ENABLE, "Torque")
        if torque is not None:
            print(f"  Torque Enable         : {'ON' if torque else 'OFF'}")

        print()

    print("=" * 55)
    port.closePort()
    print("Done.")


if __name__ == "__main__":
    main()
