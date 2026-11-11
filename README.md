# Virtual Multi-Axis Paper Cutter Controller & Windows HMI

A real-time multi-axis industrial motion controller and desktop Human-Machine Interface (HMI) simulation inspired by precision commercial guillotine paper-cutting machinery.

The system features:
- A deterministic, platform-independent **C++17 real-time firmware controller core** (zero dynamic heap allocation on the 1 ms tick path, zero OS headers in `core/`, bounds-checked buffer operations, no exceptions).
- A robust, table-driven **Finite State Machine (FSM)** with guard evaluation, strict fault latching, and single-tick ($\le 1\text{ ms}$) emergency deceleration halts.
- Discrete fixed-point **trapezoidal motion planning** with discrete stopping distance calculation $s_{\text{stop}}(v) = \frac{v(v + dv)}{2a}$ preventing velocity discontinuities.
- A multi-language **binary packet protocol** with SOF framing (`0xAA 0x55`), CRC-16/CCITT-FALSE error checking, and verified identical golden vectors across C++, C#, and Python.
- A modern **WPF desktop HMI application (.NET 8)** adhering strictly to MVVM (`CommunityToolkit.Mvvm`), responsive background asynchronous I/O, live telemetry instrumentation, recipe editing, and a dedicated, always-enabled hardware Emergency Stop (E-Stop).
- An end-to-end automated **verification test suite** (T01–T11) generating JUnit XML test reports.

---

## Architecture Overview

```
 +-------------------------------------------------------------------------+
 |                          Desktop Windows HMI                            |
 |                     (C# .NET 8 / WPF / MVVM)                            |
 |                                                                         |
 |  [ MainViewModel ] <-------- CommunicationService (Background Worker)   |
 |       |                                    |                            |
 |  [ UI Controls ]                           v                            |
 |  - Big Red E-STOP (Bypass Queue)     PacketEncoder / PacketParser       |
 |  - Live Axis Gauges (20 Hz)          (Big-Endian Wire Codec)            |
 |  - Recipe DataGrid & Interlocks            |                            |
 +--------------------------------------------|----------------------------+
                                              |
                     Win32 Serial (COM10<->COM11 via com0com)
                                     or TCP Loopback (127.0.0.1)
                                              |
 +--------------------------------------------v----------------------------+
 |                      Controller Application Binary                      |
 |                      (controller_app / CMake C++17)                     |
 |                                                                         |
 |   WinSerialTransport / TcpTransport / MemoryTransport (ITransport)     |
 |                                    |                                    |
 |   +--------------------------------v--------------------------------+   |
 |   |                 Platform-Independent Core (core/)              |   |
 |   |                                                                |   |
 |   |   - Streaming Packet Parser & Encoder                          |   |
 |   |   - Table-Driven Finite State Machine (fsm::StateMachine)      |   |
 |   |   - Safety Supervisor (Watchdog, CRC Storm, Overrun Bounds)    |   |
 |   |   - Trapezoidal Motion Planner & Blade Kinematics              |   |
 |   |   - Simulated Hardware (Pneumatic Clamp, Limit Switches, Jam)  |   |
 |   |   - 20 Hz Telemetry Broadcast Stream (CMD 0x30)                |   |
 |   +----------------------------------------------------------------+   |
 +-------------------------------------------------------------------------+
```

---

## Prerequisites & Environment

- **Operating System**: Windows 10 or Windows 11 (64-bit)
- **C++ Compiler & Build**: Visual Studio 2022 (MSVC x64, C++17 support) and CMake $\ge 3.20$
- **.NET SDK**: .NET 8.0 SDK (or .NET 9.0)
- **Python**: Python 3.11+ (with `pytest` and `pyserial`)
- **Virtual COM Port Driver** (Optional for serial mode): [Null-modem emulator (com0com)](https://sourceforge.net/projects/com0com/) configured with a pair linking `COM10` and `COM11`.

---

## Repository Structure

```
├── core/                         # Platform-independent C++17 core (No OS headers)
│   ├── include/core/
│   │   ├── protocol/             # Packet definition, CRC-16 table, encoder/parser
│   │   ├── fsm/                  # Table-driven state machine & transition matrix
│   │   ├── motion/               # Trapezoidal profile & cyclic blade planners
│   │   ├── safety/               # Watchdog, CRC storm detector, limit monitor
│   │   ├── sim/                  # Simulated hardware (clamp, TDC/BDC, feed sensor)
│   │   ├── itransport.hpp        # Pure virtual transport interface
│   │   └── iclock.hpp            # Pure virtual clock interface
│   └── src/                      # Core C++17 implementations
├── controller_app/               # Controller application executable
│   ├── transport_serial_win.cpp  # Win32 serial COM port transport (COM10)
│   ├── transport_tcp.cpp         # Loopback TCP socket transport
│   ├── transport_mem.cpp         # In-memory FIFO transport for unit tests
│   └── main.cpp                  # 1 ms tick control loop & 20 Hz telemetry
├── hmi/                          # Windows Presentation Foundation (WPF) HMI
│   ├── PaperCutter.Hmi/          # .NET 8 WPF App (CommunityToolkit.Mvvm)
│   │   ├── Protocol/             # C# wire codec & CRC-16/CCITT-FALSE
│   │   ├── Services/             # Non-blocking communication worker & queues
│   │   ├── ViewModels/           # MainViewModel with telemetry & command relays
│   │   ├── Models/               # Cut recipe models
│   │   └── MainWindow.xaml       # Industrial dark dashboard & big E-stop
│   └── PaperCutter.Hmi.Tests/    # C# xUnit protocol & golden vector verification
├── tests/
│   ├── cpp/                      # GoogleTest suite (33 unit tests)
│   ├── python/                   # pytest system verification suite (T01 - T11)
│   │   ├── papercutter_protocol/ # Python protocol mirror
│   │   ├── controller_client.py  # Automation harness client (TCP/Serial)
│   │   └── test_suite.py         # Complete T01-T11 test implementation
│   └── test_vectors/
│       └── golden_vectors.json   # Golden test vectors for cross-language validation
├── demo/
│   ├── demo.py                   # Automated end-to-end demonstration script
│   └── run_demo.bat              # One-click Windows batch launcher
├── docs/
│   ├── protocol.md               # Binary packet format & telemetry spec
│   ├── state_machine.md          # State transition matrix & safety latching
│   ├── test_plan.md              # Test matrix & acceptance criteria
│   └── decisions.md              # Architectural & safety engineering decisions
├── CMakeLists.txt                # Root CMake project file
└── README.md                     # Project documentation & execution guide
```

---

## Build Instructions

### 1. Build C++ Controller & Unit Tests

Open a PowerShell terminal in the repository root:

```powershell
# Configure CMake with Visual Studio 2022 MSVC x64
cmake -B build -G "Visual Studio 17 2022" -A x64

# Build controller_app.exe and unit_tests.exe
cmake --build build --config Debug
```

The resulting binaries will be placed in:
- `build/controller_app/Debug/controller_app.exe`
- `build/tests/cpp/Debug/unit_tests.exe`

### 2. Build C# WPF HMI & xUnit Tests

```powershell
# Restore and build HMI solution
dotnet build hmi/PaperCutter.sln

# Run C# protocol unit tests
dotnet test hmi/PaperCutter.Hmi.Tests
```

The WPF executable is generated at:
- `hmi/PaperCutter.Hmi/bin/Debug/net8.0-windows/PaperCutter.Hmi.exe`

---

## Running Verification Tests

### 1. C++ GoogleTest Suite (33 Tests)

Runs all unit tests for CRC-16, streaming packet parser/encoder, golden vectors, state machine transitions, kinematics stopping distance, simulation hardware, and safety supervisor:

```powershell
.\build\tests\cpp\Debug\unit_tests.exe
```

*Result:* `[  PASSED  ] 33 tests.`

### 2. C# xUnit Test Suite (4 Tests)

Verifies identical CRC-16 calculation and bidirectional packet encoding/parsing against golden vectors:

```powershell
dotnet test hmi/PaperCutter.Hmi.Tests
```

*Result:* `Passed! - Failed: 0, Passed: 4, Skipped: 0, Total: 4`

### 3. Python System Test Suite T01–T11 (11 Tests)

Verifies full system-level execution, protocol fault injection, timing bounds, emergency stops, and deterministic latching:

```powershell
python -m pytest tests/python/test_suite.py -v --junitxml=junit.xml
```

---

## Verification Test Matrix (T01 – T11)

All 11 mandatory system verification tests defined in the specification have been implemented and pass 100%:

| Test ID | Test Name | Scenario / Fault Injected | Acceptance Criteria & Verified Behavior | Bound | Result |
|:---:|---|---|---|:---:|:---:|
| **T01** | `test_t01_normal_5_cut_job` | Normal 5-cut production job | All 5 cuts executed in order; transitions `IDLE->FEEDING->CLAMPING->CUTTING`; returns to `IDLE` with `fault_code == 0`, 0 CRC errors, 0 dropped packets. | $< 25.0\text{ s}$ | **PASS** |
| **T02** | `test_t02_paper_jam_during_feeding` | Paper jam injected during `FEEDING` | Feed sensor stall detected; jam timer trips at 3.0 s; transitions to `ERROR_JAM` (`0x0004`); `latched_fault == 0x0001`; motion halted within 1 ms. | $< 3.4\text{ s}$ | **PASS** |
| **T03** | `test_t03_dropped_packets` | 3 consecutive packets dropped via `SIM_INJECT` | Client detects unacknowledged packet, retransmits sequence; controller processes retransmission without state corruption. | $< 2.0\text{ s}$ | **PASS** |
| **T04** | `test_t04_corrupted_crc` | Frame with invalid CRC transmitted | Controller rejects frame, increments `crc_errors` counter in telemetry; state and recipe progress remain unaffected. | $< 1.0\text{ s}$ | **PASS** |
| **T05** | `test_t05_heartbeat_stops_mid_cut` | Heartbeat ceases during blade cut cycle | 200 ms watchdog expires; transitions to `ERROR_FAULT`; latches `FaultCode::COMM_TIMEOUT` (`0x0002`); motors halted. | $< 250\text{ ms}$ | **PASS** |
| **T06** | `test_t06_blade_limit_switch_fires_early` | Blade TDC switch trips during `IDLE`/`FEEDING` | Safety supervisor detects invalid limit trip; immediate halt; transitions to `ERROR_FAULT`; latches `PREMATURE_LIMIT` (`0x0003`). | $< 50\text{ ms}$ | **PASS** |
| **T07** | `test_t07_estop_during_cutting` | E-stop asserted while blade is moving down | Emergency deceleration profile executed within 1 ms tick; latches `FaultCode::ESTOP` (`0x0004`); blade safely halted. | $< 50\text{ ms}$ | **PASS** |
| **T08** | `test_t08_start_while_in_error` | `START` commanded while in `ERROR_JAM`/`ERROR_FAULT` | FSM guard detects illegal transition; command rejected with `NACK` frame; machine remains strictly latched in error state. | $< 100\text{ ms}$ | **PASS** |
| **T09** | `test_t09_reset_fault_while_sensor_unsafe` | `RESET_FAULT` commanded while sensor blocked | Guard condition detects active obstruction; rejects reset with `NACK`; fault latch preserved. | $< 100\text{ ms}$ | **PASS** |
| **T10** | `test_t10_garbage_bytes_resync` | 10,000 random noise bytes streamed over transport | Byte-wise streaming parser discards garbage without buffer overflow; resynchronises on next valid SOF `0xAA 0x55`; answers subsequent `HELLO`. | $< 2.0\text{ s}$ | **PASS** |
| **T11** | `test_t11_determinism` | Two independent runs with identical job parameters | Both runs produce identical ordered sequences of FSM state transitions and cut indices. | Deterministic | **PASS** |

---

## Operating Instructions & Demo

### Option A: Automated End-to-End Demo Script

The easiest way to observe the system is the automated demonstration script:

```powershell
python demo/demo.py
```
*(Or double-click `demo\run_demo.bat` in Windows Explorer).*

The demo will:
1. Launch the WPF HMI and Controller simulation.
2. Complete the protocol handshake (`HELLO`) and maintain the 100 ms heartbeat.
3. Load a 3-cut production recipe (80.0 mm, 50.0 mm, 20.0 mm).
4. Initiate automatic cutting cycle (`START`).
5. Successfully complete Cut 1.
6. Inject a paper feed jam during Cut 2.
7. Demonstrate deterministic 1 ms safety halt into `ERROR_JAM` with latched fault `PAPER_JAM (0x0001)`.
8. Confirm illegal `START` command rejection with `NACK`.
9. Clear sensor obstruction and execute safe `RESET_FAULT` back to `IDLE`.

### Option B: Interactive Operation via WPF HMI

1. **Configure Serial COM Ports (Optional)**:
   - In com0com, configure a virtual pair: `COM10 <-> COM11`.
2. **Start the Controller**:
   ```powershell
   # Serial mode (COM10 at 115200 baud):
   .\build\controller_app\Debug\controller_app.exe --serial COM10 --baud 115200

   # Or TCP loopback mode:
   .\build\controller_app\Debug\controller_app.exe --tcp 9000
   ```
3. **Launch the HMI**:
   ```powershell
   .\hmi\PaperCutter.Hmi\bin\Debug\net8.0-windows\PaperCutter.Hmi.exe
   ```
4. **Connect & Run**:
   - In the HMI header bar, select `COM11` (or check `Use TCP` with port 9000) and click **Connect**.
   - Observe the live 20 Hz telemetry indicators (Back-gauge gauge, Blade stroke, optical feed sensor, TDC/BDC limit switches).
   - Click **1. LOAD RECIPE** to transmit the cut job.
   - Click **2. START CYCLE** to run the job.
   - Hit the **BIG RED EMERGENCY STOP** button at any time to verify instantaneous hardware safety shutdown!

---

## Engineering Compliance & Design Rules

- **Zero Dynamic Allocation in Tick Path**: All data buffers, state transition tables, and packet buffers use fixed-size preallocated storage.
- **Platform-Independent Core**: `core/` contains no `<windows.h>`, POSIX headers, or dynamic memory (`new`/`delete`).
- **Deterministic 1 ms Execution**: Fixed-point motion profiles and 256-word precomputed CRC-16 table ensure all tick computations finish well within sub-millisecond real-time budgets.
- **Fail-Safe Interlocks**: Faults latch irreversibly until explicitly cleared by an operator through an authenticated `RESET_FAULT` command with sensors in verified safe states.
- **Simulation Disclaimer**: This software is a high-fidelity virtual simulation inspired by commercial paper cutters and is designed for desktop validation, automated testing, and educational research.
