"""
Sphero RVR API v2 packet building and parsing.
BLE transport — handle 0x000e, CCCD 0x000f.
"""
import struct


SOP = 0x8d
EOP = 0xd8

# Flags
FLAG_RESPONSE     = 0x01
FLAG_REQ_RESPONSE = 0x02
FLAG_ACTIVITY     = 0x08
FLAG_HAS_TARGET   = 0x10
FLAG_HAS_SOURCE   = 0x20

# Targets
PRIMARY   = 0x01   # Nordic — power/system
SECONDARY = 0x02   # STM32  — drive/sensors/LEDs

# Devices
DEV_POWER  = 0x13
DEV_DRIVE  = 0x16
DEV_SENSOR = 0x18
DEV_LED    = 0x1a

# Power commands
CMD_WAKE  = 0x0d
CMD_SLEEP = 0x01
CMD_BATT  = 0x10

# Drive commands
CMD_RAW_MOTORS       = 0x01
CMD_RESET_YAW        = 0x06
CMD_DRIVE_HEADING    = 0x07

# Sensor streaming commands
CMD_SENSOR_CONFIG    = 0x39
CMD_SENSOR_START     = 0x3a
CMD_SENSOR_STOP      = 0x3b
CMD_SENSOR_CLEAR     = 0x3c
CMD_SENSOR_STREAM    = 0x3d   # async notify cmd id

# LED command
CMD_SET_ALL_LEDS     = 0x1a

# Sensor IDs (must match SDK: sensor_streaming_control.py __init_services)
SENSOR_IMU       = 0x0001   # Pitch, Roll, Yaw  (deg)  — token 1, ST
SENSOR_ACCEL     = 0x0002   # X, Y, Z  (G)             — token 1, ST
SENSOR_GYRO      = 0x0004   # X, Y, Z  (deg/s)         — token 1, ST
SENSOR_LOCATOR   = 0x0006   # X, Y  (cm)               — token 2, ST
SENSOR_VELOCITY  = 0x0007   # X, Y  (m/s)              — token 2, ST

# Data size enum value (StreamingDataSizesEnum.thirty_two_bit = 0x02)
DATA_SIZE_32BIT = 0x02

# Number of attributes per sensor
SENSOR_ATTRS = {
    SENSOR_IMU:      3,   # Pitch, Roll, Yaw
    SENSOR_ACCEL:    3,   # X, Y, Z
    SENSOR_GYRO:     3,   # X, Y, Z
    SENSOR_LOCATOR:  2,   # X, Y
    SENSOR_VELOCITY: 2,   # X, Y
}

# Value ranges for normalization (uint32 → float)
# normalize(uint32_val, 0, UINT32_MAX, range_min, range_max)
UINT32_MAX = 0xFFFFFFFF
SENSOR_RANGES = {
    SENSOR_IMU:      [(-180.0, 180.0), (-90.0, 90.0), (-180.0, 180.0)],
    SENSOR_ACCEL:    [(-16.0, 16.0)] * 3,
    SENSOR_GYRO:     [(-2000.0, 2000.0)] * 3,
    SENSOR_LOCATOR:  [(-2147483648.0, 2147483647.0)] * 2,
    SENSOR_VELOCITY: [(-2147483648.0, 2147483647.0)] * 2,
}


def normalize_uint32(raw, range_min, range_max):
    """Map uint32 [0, UINT32_MAX] → [range_min, range_max]."""
    return range_min + raw * (range_max - range_min) / UINT32_MAX


def checksum(payload):
    return (~sum(payload)) & 0xff


def build_packet(flags, target, device, cmd, seq, data=b''):
    payload = bytes([flags, target, device, cmd, seq]) + data
    chk = checksum(payload)
    return bytes([SOP]) + payload + bytes([chk, EOP])


def parse_packet(raw):
    """Parse a raw notification bytes.
    Returns (flags, target, device, cmd, seq, payload) or None on error.
    """
    if len(raw) < 6:
        return None
    if raw[0] != SOP or raw[-1] != EOP:
        return None

    inner = raw[1:-1]   # flags ... chk
    if checksum(inner[:-1]) != inner[-1]:
        return None     # bad checksum

    body = inner[:-1]   # flags ... data
    idx = 0
    flags = body[idx]; idx += 1
    target = None
    if flags & FLAG_HAS_TARGET:
        target = body[idx]; idx += 1
    source = None
    if flags & FLAG_HAS_SOURCE:
        source = body[idx]; idx += 1
    if len(body) < idx + 3:
        return None
    device = body[idx]; idx += 1
    cmd    = body[idx]; idx += 1
    seq    = body[idx]; idx += 1
    payload = body[idx:]
    return flags, target, device, cmd, seq, payload


def unpack_floats(data, n):
    """Unpack n big-endian float32 values from bytes."""
    if len(data) < n * 4:
        return None
    return struct.unpack('>' + 'f' * n, data[:n * 4])
