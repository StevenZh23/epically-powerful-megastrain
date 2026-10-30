"""Raw CAN-FD register-protocol helpers for mjbots moteus controllers.

Implements a subset of the moteus multiplex register protocol sufficient for
real-time position / velocity / torque control and state feedback over
socketcan CAN-FD (1 Mbps arbitration / 5 Mbps data).

Protocol reference: https://mjbots.github.io/moteus/protocol/can/
"""

from __future__ import annotations

import math
import struct
from typing import Optional

import can

# ~~~~~ Registers ~~~~~ #
REG_MODE = 0x000
REG_POSITION = 0x001
REG_VELOCITY = 0x002
REG_TORQUE = 0x003
REG_VOLTAGE = 0x00D
REG_TEMPERATURE = 0x00E
REG_FAULT = 0x00F

REG_COMMAND_POSITION = 0x020
REG_COMMAND_VELOCITY = 0x021
REG_COMMAND_FEEDFORWARD_TORQUE = 0x022
REG_COMMAND_KP_SCALE = 0x023
REG_COMMAND_KD_SCALE = 0x024
REG_COMMAND_ILIMIT_SCALE = 0x02B

REG_SET_OUTPUT_NEAREST = 0x130
REG_SET_OUTPUT_EXACT = 0x131

# ~~~~~ Modes ~~~~~ #
MODE_STOPPED = 0
MODE_FAULT = 1
MODE_POSITION = 10
MODE_TIMEOUT = 11

# ~~~~~ Multiplex subframe opcodes ~~~~~ #
WRITE_INT8 = 0x00
WRITE_F32 = 0x0C
READ_INT8 = 0x10
READ_F32 = 0x1C
NOP = 0x50

# Default host source ID. Bit 7 is set in the CAN ID when a reply is requested.
MASTER_CAN_ID = 0

_FD_LENGTHS = (0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64)


def _write_varuint(buf: bytearray, value: int) -> None:
    while True:
        byte = value & 0x7F
        value >>= 7
        if value:
            buf.append(byte | 0x80)
        else:
            buf.append(byte)
            return


def _read_varuint(data: bytes, offset: int) -> tuple[Optional[int], int]:
    result = 0
    shift = 0
    for i in range(5):
        if offset + i >= len(data):
            return None, offset + i
        this_byte = data[offset + i]
        result |= (this_byte & 0x7F) << shift
        shift += 7
        if (this_byte & 0x80) == 0:
            return result, offset + i + 1
    return None, offset + 5


def _pad_fd(data: bytes) -> bytes:
    """Pad multiplex payload to a legal CAN-FD DLC with moteus NOP bytes."""
    n = len(data)
    for length in _FD_LENGTHS:
        if length >= n:
            if length == n:
                return data
            return data + bytes([NOP] * (length - n))
    return data[:64]


def build_arbitration_id(target_id: int, source_id: int = MASTER_CAN_ID, reply: bool = True) -> int:
    """Build the 16-bit moteus CAN ID: (source << 8) | destination.

    When ``reply`` is True, bit 7 of the source field is set so the controller
    responds even if the frame also contains query subframes.
    """
    source = (source_id & 0x7F) | (0x80 if reply else 0x00)
    return ((source & 0xFF) << 8) | (target_id & 0x7F)


def reply_source_id(arbitration_id: int) -> int:
    """Extract the responding moteus servo ID from a reply frame ID."""
    return (arbitration_id >> 8) & 0x7F


def _append_write_int8(buf: bytearray, start_register: int, values: list[int]) -> None:
    count = len(values)
    if count <= 0:
        return
    if count <= 3:
        buf.append(WRITE_INT8 | count)
    else:
        buf.append(WRITE_INT8)
        buf.append(count)
    _write_varuint(buf, start_register)
    for value in values:
        buf.append(struct.pack("<b", int(value))[0])


def _append_write_f32(buf: bytearray, start_register: int, values: list[float]) -> None:
    count = len(values)
    if count <= 0:
        return
    if count <= 3:
        buf.append(WRITE_F32 | count)
    else:
        buf.append(WRITE_F32)
        buf.append(count)
    _write_varuint(buf, start_register)
    for value in values:
        buf.extend(struct.pack("<f", float(value)))


def _append_standard_query(buf: bytearray) -> None:
    """Query mode (int8), position/velocity/torque (f32), voltage/temp/fault (int8)."""
    buf.append(READ_INT8 | 1)
    _write_varuint(buf, REG_MODE)
    buf.append(READ_F32 | 3)
    _write_varuint(buf, REG_POSITION)
    buf.append(READ_INT8 | 3)
    _write_varuint(buf, REG_VOLTAGE)


def _make_message(target_id: int, payload: bytes, reply: bool = True) -> can.Message:
    arbitration_id = build_arbitration_id(target_id, reply=reply)
    data = _pad_fd(payload)
    return can.Message(
        arbitration_id=arbitration_id,
        data=data,
        is_extended_id=arbitration_id > 0x7FF,
        is_fd=True,
        bitrate_switch=True,
        check=True,
    )


def create_stop_message(target_id: int, query: bool = True) -> can.Message:
    """Enter stopped mode (clears faults). Optionally request a state reply."""
    buf = bytearray()
    _append_write_int8(buf, REG_MODE, [MODE_STOPPED])
    if query:
        _append_standard_query(buf)
    return _make_message(target_id, bytes(buf), reply=query)


def create_query_message(target_id: int) -> can.Message:
    buf = bytearray()
    _append_standard_query(buf)
    return _make_message(target_id, bytes(buf), reply=True)


def create_position_message(
    target_id: int,
    position: float,
    velocity: float = 0.0,
    feedforward_torque: float = 0.0,
    kp_scale: float = 1.0,
    kd_scale: float = 1.0,
    ilimit_scale: Optional[float] = None,
    query: bool = True,
) -> can.Message:
    """Command position mode.

    Units are moteus native: position in revolutions, velocity in rev/s,
    torque in N·m, kp/kd/ilimit as unitless scales of the onboard PID.
    Pass ``math.nan`` for position to mean "wherever you are now".
    """
    buf = bytearray()
    _append_write_int8(buf, REG_MODE, [MODE_POSITION])

    # Registers 0x020-0x024 are contiguous; write them as one float block.
    _append_write_f32(
        buf,
        REG_COMMAND_POSITION,
        [position, velocity, feedforward_torque, kp_scale, kd_scale],
    )
    if ilimit_scale is not None:
        _append_write_f32(buf, REG_COMMAND_ILIMIT_SCALE, [ilimit_scale])

    if query:
        _append_standard_query(buf)
    return _make_message(target_id, bytes(buf), reply=query)


def create_torque_message(
    target_id: int,
    torque: float,
    query: bool = True,
) -> can.Message:
    """Pure feedforward torque: position=NaN, velocity=0, PID scales=0."""
    return create_position_message(
        target_id,
        position=math.nan,
        velocity=0.0,
        feedforward_torque=torque,
        kp_scale=0.0,
        kd_scale=0.0,
        ilimit_scale=0.0,
        query=query,
    )


def create_set_output_exact_message(
    target_id: int,
    position: float = 0.0,
    query: bool = True,
) -> can.Message:
    """Force reported output position to ``position`` (revolutions)."""
    buf = bytearray()
    _append_write_f32(buf, REG_SET_OUTPUT_EXACT, [position])
    if query:
        _append_standard_query(buf)
    return _make_message(target_id, bytes(buf), reply=query)


def create_set_output_nearest_message(
    target_id: int,
    position: float = 0.0,
    query: bool = True,
) -> can.Message:
    """Snap reported output position to the nearest whole rotation of ``position``."""
    buf = bytearray()
    _append_write_f32(buf, REG_SET_OUTPUT_NEAREST, [position])
    if query:
        _append_standard_query(buf)
    return _make_message(target_id, bytes(buf), reply=query)


def _scale_register(reg: int, res_code: int, raw: float) -> float:
    """Apply moteus fixed-point scaling for non-float resolutions."""
    if res_code == 3:  # F32
        return float(raw)

    if reg == REG_POSITION:
        return float(raw) * (0.01, 0.0001, 0.00001)[res_code]
    if reg == REG_VELOCITY:
        return float(raw) * (0.1, 0.00025, 0.00001)[res_code]
    if reg == REG_TORQUE:
        return float(raw) * (0.5, 0.01, 0.001)[res_code]
    if reg == REG_VOLTAGE:
        return float(raw) * (0.5, 0.1, 0.001)[res_code]
    if reg == REG_TEMPERATURE:
        return float(raw) * (1.0, 0.1, 0.001)[res_code]
    return float(raw)


def parse_reply(data: bytes) -> dict[int, float]:
    """Parse moteus reply subframes into ``{register: value}``.

    Applies moteus integer scaling when values arrive as int8/int16/int32.
    Float replies are returned as-is. Trailing NOP padding is ignored.
    """
    values: dict[int, float] = {}
    offset = 0
    n = len(data)
    type_sizes = (1, 2, 4, 4)
    type_fmt = ("<b", "<h", "<i", "<f")

    while offset < n:
        cmd = data[offset]
        offset += 1
        if cmd == NOP:
            continue

        upper = cmd & 0xF0
        res_code = (cmd & 0x0C) >> 2
        maybe_num = cmd & 0x03

        if cmd in (0x30, 0x31):
            _reg, offset = _read_varuint(data, offset)
            _err, offset = _read_varuint(data, offset)
            continue

        # Only consume REPLY_* subframes (0x20 / 0x24 / 0x28 / 0x2c)
        if upper != 0x20:
            break

        if maybe_num > 0:
            length = maybe_num
        else:
            length, offset = _read_varuint(data, offset)
            if length is None:
                break

        start_reg, offset = _read_varuint(data, offset)
        if start_reg is None:
            break

        size = type_sizes[res_code]
        fmt = type_fmt[res_code]
        for i in range(length):
            if offset + size > n:
                return values
            raw = struct.unpack_from(fmt, data, offset)[0]
            offset += size
            values[start_reg + i] = _scale_register(start_reg + i, res_code, raw)

    return values
