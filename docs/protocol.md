# Binary Packet Protocol Specification

## 1. Framing Overview

All multi-byte fields are transmitted in **Big-Endian** (network byte order).
Packets begin with a 2-byte Start of Frame (SOF) sequence and terminate with a 2-byte CRC-16.

```
+-------------+------------+------------+------------+--------------------+-------------+
| SOF (2B)    | LEN (1B)   | SEQ (1B)   | CMD (1B)   | PAYLOAD (0..248B)  | CRC16 (2B)  |
| 0xAA 0x55   | 2..250     | 0..255     | see below  | Command data       | CCITT-FALSE |
+-------------+------------+------------+------------+--------------------+-------------+
|<----------- CRC-16 Calculation Range ---------------------------------->|
```

- **SOF**: `0xAA 0x55` (Fixed 2 bytes)
- **LEN**: Length of `[SEQ (1) + CMD (1) + PAYLOAD (N)]`. Minimum = 2, Maximum = 250.
- **SEQ**: Rolling 8-bit sequence number (0..255).
- **CMD**: 8-bit Command ID.
- **PAYLOAD**: 0 to 248 bytes of command-specific data. Total packet size on wire: `5 + LEN` bytes (7 to 255 bytes).
- **CRC16**: 16-bit CRC computed over `LEN`, `SEQ`, `CMD`, and `PAYLOAD`. Transmitted MSB first, then LSB.

## 2. CRC-16 Specification

- **Algorithm**: CRC-16/CCITT-FALSE
- **Polynomial**: `0x1021` ($x^{16} + x^{12} + x^5 + 1$)
- **Initial Value**: `0xFFFF`
- **Reflect In**: `False`
- **Reflect Out**: `False`
- **XOR Out**: `0x0000`
- **Check String**: ASCII `"123456789"` (bytes `0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39`) produces CRC `0x29B1`.

## 3. Command Definitions

| CMD | Name | Direction | Payload Structure |
|---|---|---|---|
| `0x01` | `HELLO / VERSION` | Both | `protocol_version (u8)`, `firmware_version (u16)` |
| `0x02` | `HEARTBEAT` | HMI -> Ctrl | None (must arrive at least every 200 ms once connected) |
| `0x10` | `LOAD_JOB` | HMI -> Ctrl | `job_id (u16)`, `cut_count (u8)`, `cuts[N] (u32 each, 0.01 mm units)` |
| `0x11` | `START` | HMI -> Ctrl | `job_id (u16)` |
| `0x12` | `PAUSE_RESUME` | HMI -> Ctrl | None |
| `0x13` | `ABORT` | HMI -> Ctrl | None |
| `0x20` | `ESTOP` | HMI -> Ctrl | None (highest priority, immediate motor halt) |
| `0x21` | `RESET_FAULT` | HMI -> Ctrl | None (only valid when sensors are safe and no E-stop active) |
| `0x30` | `TELEMETRY` | Ctrl -> HMI | See Section 4 (sent at 20 Hz / 50 ms) |
| `0x7E` | `ACK` | Ctrl -> HMI | `acked_seq (u8)`, `status (u8)` |
| `0x7F` | `NACK` | Ctrl -> HMI | `rejected_seq (u8)`, `reason (u8)` |
| `0x90` | `SIM_INJECT` | Test -> Ctrl | `fault_type (u8)`, `param (u16)` |

## 4. Telemetry Payload Format (CMD 0x30)

Total payload length = 23 bytes.

| Offset | Type | Field | Description |
|---|---|---|---|
| 0 | `u8` | `state` | Current FSM state ID |
| 1 | `u16` | `fault_code` | Latched fault code (0x0000 = none) |
| 3 | `i32` | `back_gauge_pos` | Back gauge position in 0.01 mm units |
| 7 | `i32` | `blade_pos` | Blade position in 0.01 mm units |
| 11 | `u8` | `feed_sensor` | Optical paper feed sensor (1 = paper present, 0 = clear) |
| 12 | `u8` | `limit_switches`| Bitmask: Bit 0 = Blade Top Dead Centre (TDC), Bit 1 = Blade Bottom Dead Centre (BDC), Bit 2 = Clamp Switch, Bit 3 = Safety Guard / Estop physical switch |
| 13 | `u8` | `cut_index` | Current cut index in loaded job (0-based) |
| 14 | `u16` | `crc_errors` | Cumulative CRC error counter |
| 16 | `u16` | `dropped_packets`| Cumulative dropped packets counter |
| 18 | `u16` | `timeout_errors`| Inter-byte or link timeout counter |
| 20 | `u8` | `clamp_engaged` | Clamp status (1 = engaged, 0 = retracted) |
| 21 | `u16` | `reserved` | Reserved for alignment/future flags |

## 5. Fault Codes

| Code | Name | Description |
|---|---|---|
| `0x0000` | `NONE` | No fault |
| `0x0001` | `PAPER_JAM` | Paper feed jam (sensor unchanged > 3.0 s during feeding) |
| `0x0002` | `COMM_TIMEOUT` | Heartbeat lost (> 200 ms timeout) |
| `0x0003` | `PREMATURE_LIMIT` | Blade limit switch tripped unexpectedly |
| `0x0004` | `ESTOP` | Emergency stop asserted |
| `0x0005` | `CRC_STORM` | CRC or framing error storm detected |
| `0x0006` | `MOTION_OVERRUN` | Motion planner step or boundary overrun |
| `0x0007` | `INVALID_JOB` | Rejected job specification (invalid bounds/positions) |

## 6. Parser State Machine

```
              +-------------+
       +----->|  WAIT_SOF1  |<----+
       |      +-------------+     | (Timeout or CRC Error)
       |             | 0xAA       |
       |      +-------------+     |
       |      |  WAIT_SOF2  |-----+
       |      +-------------+
       |             | 0x55
       |      +-------------+
       |      |  READ_LEN   |
       |      +-------------+
       |             | valid LEN (2..250)
       |      +-------------+
       |      |  READ_BODY  |
       |      +-------------+
       |             | LEN bytes read
       |      +-------------+
       |      |  READ_CRC1  |
       |      +-------------+
       |             | MSB read
       |      +-------------+
       |      |  READ_CRC2  |
       |      +-------------+
       |             | LSB read & verify CRC
       +-------------+
```
Inter-byte timeout threshold is 50 ms. Any timeout resets parser state to `WAIT_SOF1` and increments the timeout counter.
