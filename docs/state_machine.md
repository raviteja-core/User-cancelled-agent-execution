# Finite State Machine Specification

## 1. Machine States

| State | Enum Value | Description |
|---|---|---|
| `IDLE` | 0 | Machine stationary, awaiting job configuration and START command |
| `FEEDING` | 1 | Back-gauge moving paper to commanded cut position |
| `CLAMPING` | 2 | Clamp motor driving down to hold paper stack |
| `CUTTING` | 3 | Blade executing down-stroke to Bottom Dead Centre (BDC) and returning to Top Dead Centre (TDC) |
| `ERROR_JAM` | 4 | Paper feed jam latched (sensor unchanged > 3.0 s during feeding) |
| `ERROR_FAULT` | 5 | General safety fault latched (E-stop, comms timeout, premature limit switch, overrun) |

## 2. Table-Driven Transition Matrix

| Source State | Event | Guard Conditions | Target State | Action Executed |
|---|---|---|---|---|
| `IDLE` | `START` | Job loaded, valid cut count, interlocks safe | `FEEDING` | Start back-gauge motion planner to next cut |
| `FEEDING` | `FEED_POS_REACHED`| Feed sensor confirms paper present | `CLAMPING` | Halt back-gauge, engage clamp actuator |
| `FEEDING` | `JAM_TIMEOUT` | None (timer exceeded 3.0 s) | `ERROR_JAM` | Immediate motor halt, latch FaultCode::PAPER_JAM |
| `CLAMPING` | `CLAMP_CONFIRMED` | Clamp switch active | `CUTTING` | Start blade trapezoidal velocity profile |
| `CUTTING` | `CUT_COMPLETE` | All cuts finished | `IDLE` | Job finished, release clamp, await next job |
| `CUTTING` | `CUT_COMPLETE` | More cuts remaining | `FEEDING` | Release clamp, advance cut index, start back-gauge |
| `ANY` | `ESTOP` | None | `ERROR_FAULT` | Immediate motor halt within 1 ms, latch FaultCode::ESTOP |
| `ANY` | `HEARTBEAT_TIMEOUT` | Connected (> 200 ms) | `ERROR_FAULT` | Safe motor halt, latch FaultCode::COMM_TIMEOUT |
| `ANY` | `PREMATURE_LIMIT` | State != CUTTING | `ERROR_FAULT` | Safe motor halt, latch FaultCode::PREMATURE_LIMIT |
| `ERROR_*` | `RESET_FAULT` | Sensors safe, E-stop clear | `IDLE` | Clear latched fault, reset job counters |

## 3. Latching & Safety Halt Rules
- **Fault Latching**: Once entering `ERROR_JAM` or `ERROR_FAULT`, the state machine cannot transition to any active state except via `RESET_FAULT`.
- **Immediate Deterministic Halt**: On any safety event (`ESTOP`, `HEARTBEAT_TIMEOUT`, `PREMATURE_LIMIT`, `JAM_TIMEOUT`), motors and actuators must be instructed to halt within the current 1 ms tick.
- **Illegal Transitions**: Any `(State, Event)` pair not explicitly defined in the transition table is rejected, logged, and answers with NACK.
