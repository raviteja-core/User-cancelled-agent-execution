# Automated Verification Test Plan (T01 - T11)

This test plan defines the automated system-level test suite executed via Python (pytest) against the running controller firmware.
Each test verifies system safety, deterministic latching, error counter integrity, and timing bounds.

| Test ID | Fault Injected / Scenario | Verification & Expected Result | Timeout / Bound |
|---|---|---|---|
| **T01** | Normal 5-cut job execution | All 5 cuts executed in order, transitions through FEEDING->CLAMPING->CUTTING, returns to IDLE with fault code `0x0000`, 0 CRC errors, 0 dropped packets. | Max 10.0 s |
| **T02** | Paper jam injected during FEEDING | Feed sensor blocked; jam timeout tripped; transitions to `ERROR_JAM` (`0x0004`), `latched_fault == 0x0001` (PAPER_JAM), motors halted within 3.1 s. | Max 3.2 s |
| **T03** | Dropped packets (3 packets dropped via SIM_INJECT) | Client detects unacknowledged frame, retransmits sequence, controller processes retransmission without state corruption. | Max 2.0 s |
| **T04** | Frame with corrupted CRC transmitted | Controller rejects frame, increments `crc_errors` counter by 1, state and cut progress remain intact. | Max 1.0 s |
| **T05** | Heartbeat stops mid-cut | Heartbeat watchdog expires; transitions to `ERROR_FAULT`, latches `0x0002` (COMM_TIMEOUT), immediate motor halt within 250 ms. | < 250 ms |
| **T06** | Blade limit switch tripped prematurely while in IDLE/FEEDING | Interlock tripped; immediate safe halt, transitions to `ERROR_FAULT`, latches `0x0003` (PREMATURE_LIMIT). | < 50 ms |
| **T07** | ESTOP commanded during CUTTING | Emergency deceleration profile executed, motors halted within 1 ms tick, latches `0x0004` (ESTOP), stopping distance recorded. | < 50 ms |
| **T08** | START commanded while in error state | Controller rejects transition with NACK frame, machine remains latched in error state. | < 100 ms |
| **T09** | RESET_FAULT commanded while sensor unsafe or physical E-stop held | Controller evaluates reset guard, rejects with NACK, fault remains strictly latched. | < 100 ms |
| **T10** | 10,000 random garbage bytes streamed to serial/TCP | Byte-wise parser discards invalid bytes without buffer overrun, resynchronises on next SOF (`0xAA 0x55`), answers subsequent HELLO. | < 2.0 s |
| **T11** | Determinism verification | Two independent runs with identical job parameters and fault injection timing produce identical telemetry logs and state sequences. | Deterministic |

