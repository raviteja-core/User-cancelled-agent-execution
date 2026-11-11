#!/usr/bin/env python3
"""
Golden vector generator for the Virtual Paper Cutter binary packet protocol.
Generates shared test vectors used by C++, C#, and Python test suites.
"""

import json
import struct
from pathlib import Path


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def make_packet(seq: int, cmd: int, payload: bytes = b"") -> bytes:
    length = len(payload) + 2  # SEQ (1) + CMD (1) + payload
    header = bytes([0xAA, 0x55, length, seq, cmd])
    body_for_crc = bytes([length, seq, cmd]) + payload
    crc = crc16_ccitt_false(body_for_crc)
    crc_bytes = struct.pack(">H", crc)
    return header + payload + crc_bytes


def main():
    # Verify standard check
    standard_check = b"123456789"
    std_crc = crc16_ccitt_false(standard_check)
    assert std_crc == 0x29B1, f"Standard CRC mismatch: expected 0x29B1, got {hex(std_crc)}"

    vectors = {
        "standard_check": {
            "input_hex": standard_check.hex(),
            "expected_crc": "0x29B1",
            "expected_crc_int": 0x29B1,
        },
        "packets": [],
    }

    test_cases = [
        ("HELLO", 1, 0x01, struct.pack(">BH", 1, 100)),
        ("HEARTBEAT", 2, 0x02, b""),
        (
            "LOAD_JOB",
            3,
            0x10,
            struct.pack(">HB", 42, 3) + struct.pack(">III", 10000, 20000, 30000),
        ),
        ("START", 4, 0x11, struct.pack(">H", 42)),
        ("PAUSE", 5, 0x12, b""),
        ("ABORT", 6, 0x13, b""),
        ("ESTOP", 7, 0x20, b""),
        ("RESET_FAULT", 8, 0x21, b""),
        (
            "TELEMETRY",
            9,
            0x30,
            struct.pack(
                ">BHiiBBBHHHBH",
                1,  # state = FEEDING (1)
                0,  # fault_code = 0
                15000,  # back_gauge_pos = 15000 (150.00 mm)
                0,  # blade_pos = 0 (TDC)
                1,  # feed_sensor = paper present
                0x01,  # limit_switches = TDC active
                1,  # cut_index = 1
                0,  # crc_errors = 0
                0,  # dropped = 0
                0,  # timeouts = 0
                0,  # clamp_engaged = 0
                0,  # reserved
            ),
        ),
        ("ACK", 10, 0x7E, struct.pack(">BB", 3, 0)),
        ("NACK", 11, 0x7F, struct.pack(">BB", 3, 1)),
        ("SIM_INJECT", 12, 0x90, struct.pack(">BH", 1, 3000)),
    ]

    for name, seq, cmd, payload in test_cases:
        pkt = make_packet(seq, cmd, payload)
        body_for_crc = bytes([len(payload) + 2, seq, cmd]) + payload
        crc = crc16_ccitt_false(body_for_crc)
        vectors["packets"].append(
            {
                "name": name,
                "seq": seq,
                "cmd": cmd,
                "cmd_hex": f"0x{cmd:02X}",
                "payload_hex": payload.hex(),
                "wire_hex": pkt.hex(),
                "expected_crc": f"0x{crc:04X}",
                "expected_crc_int": crc,
            }
        )

    out_dir = Path("tests/test_vectors")
    out_dir.mkdir(parents=True, exist_ok=True)
    out_file = out_dir / "golden_vectors.json"
    with open(out_file, "w") as f:
        json.dump(vectors, f, indent=2)

    print(f"Generated {len(vectors['packets'])} golden vectors to {out_file}")


if __name__ == "__main__":
    main()
