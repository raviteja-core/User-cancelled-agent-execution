"""
Paper Cutter Binary Protocol framing, parser, and encoder.
"""

from dataclasses import dataclass, field
from enum import IntEnum
import struct
from typing import List, Optional, Tuple
from .crc16 import crc16_ccitt_false

SOF1 = 0xAA
SOF2 = 0x55
MIN_PAYLOAD_LEN = 2    # SEQ (1) + CMD (1)
MAX_PAYLOAD_LEN = 250  # SEQ (1) + CMD (1) + PAYLOAD (max 248)
INTER_BYTE_TIMEOUT_MS = 50


class Command(IntEnum):
    HELLO = 0x01
    HEARTBEAT = 0x02
    LOAD_JOB = 0x10
    START = 0x11
    PAUSE_RESUME = 0x12
    ABORT = 0x13
    ESTOP = 0x20
    RESET_FAULT = 0x21
    TELEMETRY = 0x30
    ACK = 0x7E
    NACK = 0x7F
    SIM_INJECT = 0x90


class FaultCode(IntEnum):
    NONE = 0x0000
    PAPER_JAM = 0x0001
    COMM_TIMEOUT = 0x0002
    PREMATURE_LIMIT = 0x0003
    ESTOP = 0x0004
    CRC_STORM = 0x0005
    MOTION_OVERRUN = 0x0006
    INVALID_JOB = 0x0007


class State(IntEnum):
    IDLE = 0
    FEEDING = 1
    CLAMPING = 2
    CUTTING = 3
    ERROR_JAM = 4
    ERROR_FAULT = 5


class LimitSwitch(IntEnum):
    NONE = 0x00
    BLADE_TDC = 0x01
    BLADE_BDC = 0x02
    CLAMP_SWITCH = 0x04
    SAFETY_GUARD = 0x08


@dataclass
class Packet:
    seq: int
    cmd: int
    payload: bytes = b""

    def encode(self) -> bytes:
        length = len(self.payload) + 2
        if not (MIN_PAYLOAD_LEN <= length <= MAX_PAYLOAD_LEN):
            raise ValueError(f"Invalid packet length: {length}")
        header = bytes([SOF1, SOF2, length, self.seq & 0xFF, self.cmd & 0xFF])
        body_for_crc = bytes([length, self.seq & 0xFF, self.cmd & 0xFF]) + self.payload
        crc = crc16_ccitt_false(body_for_crc)
        return header + self.payload + struct.pack(">H", crc)


@dataclass
class TelemetryData:
    state: int
    fault_code: int
    back_gauge_pos: int
    blade_pos: int
    feed_sensor: int
    limit_switches: int
    cut_index: int
    crc_errors: int
    dropped_packets: int
    timeout_errors: int
    clamp_engaged: int
    reserved: int = 0

    @classmethod
    def unpack(cls, payload: bytes) -> "TelemetryData":
        if len(payload) < 23:
            raise ValueError(f"Payload too short for telemetry: {len(payload)} < 23")
        fields = struct.unpack(">BHiiBBBHHHBH", payload[:23])
        return cls(
            state=fields[0],
            fault_code=fields[1],
            back_gauge_pos=fields[2],
            blade_pos=fields[3],
            feed_sensor=fields[4],
            limit_switches=fields[5],
            cut_index=fields[6],
            crc_errors=fields[7],
            dropped_packets=fields[8],
            timeout_errors=fields[9],
            clamp_engaged=fields[10],
            reserved=fields[11],
        )

    def pack(self) -> bytes:
        return struct.pack(
            ">BHiiBBBHHHBH",
            self.state,
            self.fault_code,
            self.back_gauge_pos,
            self.blade_pos,
            self.feed_sensor,
            self.limit_switches,
            self.cut_index,
            self.crc_errors,
            self.dropped_packets,
            self.timeout_errors,
            self.clamp_engaged,
            self.reserved,
        )


class ParserState(IntEnum):
    WAIT_SOF1 = 0
    WAIT_SOF2 = 1
    READ_LEN = 2
    READ_BODY = 3
    READ_CRC1 = 4
    READ_CRC2 = 5


class PacketParser:
    """Byte-wise streaming packet parser."""

    def __init__(self):
        self.state = ParserState.WAIT_SOF1
        self.expected_len = 0
        self.body_buf = bytearray()
        self.crc_high = 0
        self.crc_low = 0
        self.time_since_last_byte_ms = 0
        self.crc_errors = 0
        self.dropped_packets = 0
        self.timeout_errors = 0

    def reset(self):
        self.state = ParserState.WAIT_SOF1
        self.expected_len = 0
        self.body_buf.clear()
        self.crc_high = 0
        self.crc_low = 0
        self.time_since_last_byte_ms = 0

    def tick(self, delta_ms: int) -> bool:
        """Call periodically (e.g. 1 ms tick). Returns True if a timeout reset occurred."""
        if self.state != ParserState.WAIT_SOF1:
            self.time_since_last_byte_ms += delta_ms
            if self.time_since_last_byte_ms >= INTER_BYTE_TIMEOUT_MS:
                self.timeout_errors += 1
                self.reset()
                return True
        return False

    def process_byte(self, b: int) -> Optional[Packet]:
        """Processes a single incoming byte. Returns Packet if complete valid frame received."""
        self.time_since_last_byte_ms = 0

        if self.state == ParserState.WAIT_SOF1:
            if b == SOF1:
                self.state = ParserState.WAIT_SOF2
            return None

        elif self.state == ParserState.WAIT_SOF2:
            if b == SOF2:
                self.state = ParserState.READ_LEN
            elif b == SOF1:
                # Stay in WAIT_SOF2 if consecutive 0xAA
                self.state = ParserState.WAIT_SOF2
            else:
                self.state = ParserState.WAIT_SOF1
            return None

        elif self.state == ParserState.READ_LEN:
            if MIN_PAYLOAD_LEN <= b <= MAX_PAYLOAD_LEN:
                self.expected_len = b
                self.body_buf.clear()
                self.state = ParserState.READ_BODY
            else:
                self.dropped_packets += 1
                self.state = ParserState.WAIT_SOF1
            return None

        elif self.state == ParserState.READ_BODY:
            self.body_buf.append(b)
            if len(self.body_buf) >= self.expected_len:
                self.state = ParserState.READ_CRC1
            return None

        elif self.state == ParserState.READ_CRC1:
            self.crc_high = b
            self.state = ParserState.READ_CRC2
            return None

        elif self.state == ParserState.READ_CRC2:
            self.crc_low = b
            received_crc = (self.crc_high << 8) | self.crc_low

            # Calculate CRC over LEN + BODY
            calc_data = bytes([self.expected_len]) + bytes(self.body_buf)
            calculated_crc = crc16_ccitt_false(calc_data)

            if received_crc == calculated_crc:
                seq = self.body_buf[0]
                cmd = self.body_buf[1]
                payload = bytes(self.body_buf[2:])
                pkt = Packet(seq=seq, cmd=cmd, payload=payload)
                self.state = ParserState.WAIT_SOF1
                return pkt
            else:
                self.crc_errors += 1
                self.state = ParserState.WAIT_SOF1
                return None

        self.state = ParserState.WAIT_SOF1
        return None

    def parse_bytes(self, data: bytes) -> List[Packet]:
        packets = []
        for b in data:
            pkt = self.process_byte(b)
            if pkt is not None:
                packets.append(pkt)
        return packets
