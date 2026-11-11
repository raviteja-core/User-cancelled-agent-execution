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

## Decision 005: Transport Abstraction Hierarchy & Dual Mode Support
- **Context**: The controller must run on Windows with com0com (COM10/COM11), TCP for CI / loopback, and in-memory for unit tests.
- **Decision**: `ITransport` defines non-blocking `read()`, `write()`, `is_open()`, and `close()`.
  - `WinSerialTransport` uses Win32 `CreateFileW`, `SetCommState`, `SetCommTimeouts` with zero OS headers in `core/`.
  - `TcpTransport` provides loopback TCP socket connection for automated CI testing and cross-process verification.
  - `MemoryTransport` provides an in-memory paired FIFO ring buffer for deterministic unit testing.

## Decision 006: Table-Driven Finite State Machine & Guard Branching
- **Context**: Rule 6 requires table-driven state machine, rejecting illegal transitions with NACK, and latching faults.
- **Decision**: Transitions are stored in a contiguous `constexpr` array `TRANSITION_TABLE`. Multiple rows with the same `(from_state, event)` are supported for branching based on guards (e.g. `guard_cut_complete_next` vs `guard_cut_complete_done`). If any guard succeeds, the transition executes; if an event is recognized in the current state but no guard passes, `GUARD_FAILED` is returned. If the event is unrecognized for that state, `ILLEGAL_TRANSITION` is returned to trigger a NACK with precise error telemetry.
- **Fault Latching**: Any error state (`ERROR_JAM`, `ERROR_FAULT`) immediately asserts `motors_halted = true` within the current tick and latches `latched_fault`. Only `RESET_FAULT` with verified safe physical inputs can clear the fault latch.

## Decision 007: Trapezoidal Motion Profile & Discrete Stopping Distance
- **Context**: Section 1.6 requires trapezoidal velocity profile for back-gauge and cyclic blade profile with integer/fixed-point arithmetic in 1 ms tick and emergency deceleration stopping distance calculation.
- **Decision**: In discrete 1 ms time increments, stopping distance required from velocity $v$ at acceleration $a$ ($dv = a / 1000$) is computed as $s_{stop}(v) = \frac{v (v + dv)}{2a}$. When remaining distance $\le s_{stop}(v)$, the planner transitions to deceleration, stepping down velocity by exactly $dv$ per tick to arrive at target with $v=0$ without velocity discontinuity or position overshoot.
- **Emergency Stopping**: On fault or E-stop, the planner switches to emergency deceleration rate $a_{emerg}$, records distance from trigger to full stop, and never allows step inversion.

## Decision 008: Hardware Simulation & 20 Hz Telemetry Broadcast
- **Context**: Section 1.4 requires telemetry sent at 20 Hz (50 ms). Section 1.7 requires modeling back-gauge, blade limit switches (TDC/BDC), feed sensor, clamp delay, and fault injection.
- **Decision**: `SimulatedHardware` models a 150 ms clamp travel delay before the confirm switch closes, optical paper feed sensor activation, and limit switches for TDC and BDC. The controller tick loop generates and transmits `CMD 0x30` every 50 ticks (50 ms / 20 Hz) containing live position, switch states, fault latch, and error counters.

## Decision 009: Centralized Safety Supervisor & Interlocks
- **Context**: Rule 6 requires faults to halt motion within one 1 ms tick and latch safely. Section 1.4 requires watchdog (200 ms timeout), CRC storm, and motion boundary overrun protections.
- **Decision**: `SafetySupervisor` runs every 1 ms tick. It checks:
  1. E-stop assertion: immediately triggers emergency deceleration on back gauge and blade.
  2. Heartbeat watchdog: if > 200 ms without communication once connected, trips `FaultCode::COMM_TIMEOUT`.
  3. CRC Storm detection: if >= 5 CRC errors occur within 100 ms, trips `FaultCode::CRC_STORM`.
  4. Motion overrun: if position exceeds [0, 100000] steps (0..1000 mm), trips `FaultCode::MOTION_OVERRUN`.
  5. Premature limit switch: if TDC switch clears when not in `CUTTING`, trips `FaultCode::PREMATURE_LIMIT`.





