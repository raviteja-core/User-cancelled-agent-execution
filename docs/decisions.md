# Engineering Decisions Log

This document records architectural, safety, and implementation decisions made in accordance with Specification Rule 1.

## Decision 001: CRC-16/CCITT-FALSE Specification
- **Context**: Section 1.4 requires CRC-16/CCITT-FALSE (polynomial 0x1021, init 0xFFFF, no reflection in/out, no final XOR).
- **Standard Check**: Standard ASCII string `"123456789"` must produce `0x29B1`.
- **Decision**: Precomputed 256-entry lookup table is used for 1 ms tick determinism and speed. No dynamic memory allocation.

## Decision 002: Big-Endian Wire Encoding & Buffer Bounds
- **Context**: Big-endian byte order is required for all multi-byte fields.
- **Decision**: Implement explicit bit-shift serialization/deserialization functions (`write_u16_be`, `read_u16_be`, etc.). Every write operation explicitly checks remaining buffer capacity and returns an error status instead of throwing exceptions.

## Decision 003: Core Library Platform Independence & Zero Dynamic Allocation
- **Context**: Rule 3 requires `core/` to be platform-independent C++17 with no OS headers. Rule 4 forbids dynamic allocation in control tick path and exceptions in `core/`.
- **Decision**: Core headers use `<cstdint>`, `<cstddef>`, `<array>`, `<string_view>`. No `<windows.h>`, `<sys/...>`, `new`, `delete`, `malloc`, or `throw`. Max packet payload buffer size fixed at 256 bytes (`MAX_PACKET_SIZE = 260`).

## Decision 004: Byte-wise Streaming Parser State Machine & Resync
- **Context**: Section 1.4 requires byte-wise state machine (`WAIT_SOF1`, `WAIT_SOF2`, `READ_LEN`, `READ_BODY`, `READ_CRC`).
- **Decision**: Parser maintains an internal state enum:
  - `WAIT_SOF1`: wait for 0xAA
  - `WAIT_SOF2`: wait for 0x55 (if 0xAA, stay in WAIT_SOF2; otherwise back to WAIT_SOF1)
  - `READ_LEN`: read length byte (valid range: 2 <= LEN <= 250, since LEN covers SEQ[1] + CMD[1] + PAYLOAD[0..248])
  - `READ_BODY`: read LEN bytes into frame buffer
  - `READ_CRC1`: read first byte of CRC (MSB)
  - `READ_CRC2`: read second byte of CRC (LSB), verify CRC over LEN + BODY.
  On bad CRC or invalid length, increment `crc_errors` or `dropped_packets`, reset parser to `WAIT_SOF1`.
  50 ms inter-byte timeout is handled via `tick(delta_ms)`: if timed out while reading a packet, reset and increment `timeout_counter`.
