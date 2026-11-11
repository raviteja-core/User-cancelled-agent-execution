# Virtual Multi-Axis Paper Cutter Controller & Windows HMI

AI reference document

---

## Part 1: Reference Document

### 1.1 Purpose and honesty note

Build a software-only simulation of a precision paper-cutter control stack: an embedded-style controller, a WPF operator screen, and an automated fault-injection test suite. It is modelled on typical industrial cutter architecture. Do not claim it is the exact stack of any specific manufacturer; describe it as 'inspired by' that class of machine.

### 1.2 Components

1. **Controller (C++17)**: acts as the firmware. Event-driven finite state machine, motion planner, packet protocol, safety logic, telemetry.
2. **HMI (C# WPF, .NET 8, MVVM)**: connects over a COM port, sends cut jobs, shows live telemetry, has an E-stop button.
3. **Verification suite (Python + pytest, plus C++ unit tests)**: injects faults and asserts a safe, deterministic halt.

### 1.3 Repository layout

```
papercutter/
  CMakeLists.txt
  docs/                      protocol.md, state_machine.md, test_plan.md
  core/                      platform-independent C++ (no OS headers)
    protocol/                framing, CRC-16, parser, encoder
    fsm/                     state machine
    motion/                  trapezoidal planner, step scheduler
    safety/                  interlocks, watchdog
    sim/                     simulated sensors and mechanics
  controller_app/            main loop, transport adapters
    transport_serial_win.cpp COM port (Win32 CreateFile/ReadFile)
    transport_tcp.cpp        loopback for CI
    transport_mem.cpp        in-memory for unit tests
  hmi/PaperCutter.Hmi/       WPF solution (MVVM)
    Protocol/                C# mirror of the packet codec
  tests/
    cpp/                     GoogleTest unit tests
    python/                  pytest fault-injection suite
  tools/                     packet logger, golden-vector generator
```

Rule: `core/` must compile with MSVC, GCC and Clang and must never include OS headers. All I/O goes through a `ITransport` interface and a `IClock` interface (so tests control time).

### 1.4 Binary packet protocol

All multi-byte fields are **big-endian**. CRC is **CRC-16/CCITT-FALSE** (poly 0x1021, init 0xFFFF, no reflection, no final XOR) computed over every byte from `LEN` through the end of `PAYLOAD`.

| Field | Size | Description |
| --- | --- | --- |
| SOF | 2 | 0xAA 0x55 |
| LEN | 1 | Number of bytes in SEQ + CMD + PAYLOAD (max 250) |
| SEQ | 1 | Rolling sequence number, 0-255 |
| CMD | 1 | Command ID (below) |
| PAYLOAD | 0-248 | Command-specific |
| CRC16 | 2 | Over LEN, SEQ, CMD, PAYLOAD |

**Parser rules:** a byte-wise state machine (`WAIT_SOF1`, `WAIT_SOF2`, `READ_LEN`, `READ_BODY`, `READ_CRC`). On bad CRC, bad length or inter-byte timeout (50 ms), drop the frame, increment a counter, and resynchronise by searching for the next SOF. The parser must never read out of bounds on any input.

**Commands**

| CMD | Name | Direction | Payload |
| --- | --- | --- | --- |
| 0x01 | HELLO / VERSION | both | protocol version (u8), firmware version (u16) |
| 0x02 | HEARTBEAT | HMI -> ctrl | none (must arrive at least every 200 ms once connected) |
| 0x10 | LOAD\_JOB | HMI -> ctrl | job id (u16), cut count (u8), then N x cut position in 0.01 mm units (u32) |
| 0x11 | START | HMI -> ctrl | job id (u16) |
| 0x12 | PAUSE / RESUME | HMI -> ctrl | none |
| 0x13 | ABORT | HMI -> ctrl | none |
| 0x20 | ESTOP | HMI -> ctrl | none (highest priority) |
| 0x21 | RESET\_FAULT | HMI -> ctrl | none (only valid when sensors are safe) |
| 0x30 | TELEMETRY | ctrl -> HMI | state (u8), fault code (u16), back-gauge position (i32), blade position (i32), feed sensor (u8), limit switches (u8 bitmask), cut index (u8), counters (u16 each: crc errors, dropped, timeouts) |
| 0x7E | ACK | ctrl -> HMI | acked SEQ (u8), status (u8) |
| 0x7F | NACK | ctrl -> HMI | rejected SEQ (u8), reason (u8) |
| 0x90 | SIM\_INJECT | test -> ctrl | fault type (u8), parameter (u16); accepted only in simulation mode |

Every command except TELEMETRY must be answered with ACK or NACK. Telemetry is sent at 20 Hz.

**Fault codes (u16):** 0x0000 none, 0x0001 paper jam, 0x0002 communication timeout (heartbeat lost), 0x0003 premature limit switch, 0x0004 E-stop, 0x0005 CRC/frame storm, 0x0006 motion overrun, 0x0007 invalid job.

### 1.5 State machine

States: `IDLE`, `FEEDING`, `CLAMPING`, `CUTTING`, `ERROR_JAM` (generalise to a latched fault state that carries a fault code; keep the name `ERROR_JAM` for jam and add `ERROR_FAULT` or use the code field).

| From | Event | Guard | To | Action |
| --- | --- | --- | --- | --- |
| IDLE | START | job loaded, interlocks closed | FEEDING | start back-gauge move to cut position |
| FEEDING | position reached | feed sensor sees paper | CLAMPING | engage clamp |
| FEEDING | jam timeout | no sensor change in 3 s | ERROR\_JAM | halt motors, latch fault |
| CLAMPING | clamp confirmed | clamp switch closed | CUTTING | start blade profile |
| CUTTING | blade at bottom and return | limit switches in order | IDLE or FEEDING | next cut, or job complete |
| any | ESTOP | none | ERROR | immediate motor halt, clamp and blade held safe, latch |
| any | heartbeat timeout | 200 ms exceeded once connected | ERROR | same safe halt |
| any | premature blade limit switch | blade limit active when not CUTTING | ERROR | same safe halt |
| ERROR | RESET\_FAULT | all sensors safe, no E-stop | IDLE | clear latch, zero counters of the job |

Rules: transitions are table-driven and every illegal (state, event) pair is rejected with NACK and logged. A fault is **latched**: it cannot clear itself. Entering any error state must complete within one 1 ms control tick (deterministic halt).

### 1.6 Motion planner

- Control tick: 1 ms from `IClock` (simulated time in tests).
- Use a **trapezoidal velocity profile** (accelerate, cruise, decelerate; fall back to triangular if the move is short) for the back gauge and a separate blade profile.
- Output step intervals: `interval_us = 1e6 / (steps_per_second)`. Use integer or fixed-point arithmetic in the tick path; avoid floating point drift by accumulating position in steps.
- Parameters (configurable): steps/mm, max velocity, max acceleration, blade stroke, blade deceleration distance.
- **Blade deceleration:** the blade must decelerate smoothly to a stop at the bottom of the stroke and at top dead centre, never exceeding the configured decel limit. On any fault, apply the emergency decel (higher limit) rather than an instantaneous stop, and record the stopping distance.
- Unit tests must check: total steps equal commanded distance, peak velocity within limit, acceleration within limit, no negative intervals.

### 1.7 Simulated hardware

`sim/` models: the back gauge (position integrates from step output), the blade (position, top and bottom limit switches), an optical feed sensor (paper present or not), a clamp (with a confirm switch and delay), and a jam condition. A fault injector can: block paper movement (jam), drop the next N packets, corrupt a CRC, delay packets, trigger the blade limit switch early, or stop heartbeats.

### 1.8 HMI requirements (WPF, MVVM)

- Connection panel: COM port picker, baud (115200 8N1), connect/disconnect, link status, CRC error and dropped-packet counters.
- Job panel: grid to enter cut positions, validation (range, ordering), Load and Start/Pause/Abort buttons.
- Live panel: state indicator with colour, fault banner, back-gauge and blade position bars, feed sensor and limit switch lamps, cut progress.
- **E-stop button**: large and red, always enabled, sends ESTOP immediately and also bypasses any queued commands. Never blocks the UI thread.
- Background serial I/O (`SerialPort` or `System.IO.Ports` on a dedicated task), thread-safe queues, UI updates through the dispatcher. Heartbeat every 100 ms. Reconnect logic.
- Event log view with timestamps and a raw packet hex view.
- Use `CommunityToolkit.Mvvm`, `ObservableObject`, `RelayCommand`. No business logic in code-behind.

### 1.9 Verification suite

**C++ unit tests (GoogleTest):** CRC against published vectors (`123456789` -> 0x29B1), encoder/decoder round trip, parser fuzzing (random bytes, truncated frames, oversize length), every FSM transition and every illegal transition, motion planner limits.

**Python system tests (pytest + pyserial) against the running controller:**

| ID | Fault injected | Expected result |
| --- | --- | --- |
| T01 | Normal 5-cut job | All cuts done, returns to IDLE, no faults |
| T02 | Paper jam during FEEDING | ERROR\_JAM within 3.1 s, motors halted, latched |
| T03 | Dropped packets (3 in a row) | Retransmit or NACK handling, no state corruption |
| T04 | Corrupted CRC | Frame ignored, crc counter +1, state unchanged |
| T05 | Heartbeat stops mid-cut | Safe halt within 250 ms, fault 0x0002 |
| T06 | Blade limit switch fires early | Immediate halt, fault 0x0003 |
| T07 | ESTOP during CUTTING | Halt within 1 tick, blade decelerates at emergency decel, latched |
| T08 | START while in error | NACK, state unchanged |
| T09 | RESET\_FAULT while sensor unsafe | NACK, still latched |
| T10 | 10,000 random garbage bytes | Controller stays alive, resyncs, answers next valid HELLO |
| T11 | Determinism | Same seed and same fault script produce byte-identical telemetry logs |

Each test asserts the **final state, fault code, motor-halted flag and timing**. Produce a JUnit XML report and a short summary table.

### 1.10 Definition of done

- Builds from a clean clone with documented commands.
- All unit and system tests pass; a README shows the test matrix and results.
- HMI can run a full job and recover from each injected fault.
- `docs/protocol.md` matches the code exactly (golden test vectors shared by C++, C# and Python).
- A 2-minute demo script and screenshots/GIF are ready for the portfolio.

### 1.11 Suggested build order

1. Protocol library and CRC with golden vectors (C++, C#, Python).
2. Transport abstraction and loopback echo between controller and a Python script over com0com.
3. FSM with unit tests.
4. Motion planner with unit tests.
5. Simulated hardware and telemetry stream.
6. Safety logic (E-stop, watchdog, interlocks).
7. Fault injector and the pytest suite.
8. WPF HMI.
9. Polish: logging, docs, demo, CI.

---

## Part 2: Quick Interview Talking Points

- Why a transport abstraction: the same firmware core runs over a COM port, TCP, or in memory, which makes it testable and portable.
- Why latched, deterministic faults: a real machine must never resume motion on its own after a jam, an E-stop, or lost communication.
- Why CRC plus resynchronisation: UART links drop and corrupt bytes, so the parser has to survive garbage.
- What the tests prove: every injected fault ends in a known safe state within a measured time.
