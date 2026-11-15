# PC ↔ ESP32 packet protocol

USB serial, 460800 baud, 8N1. Both directions use the same frame:

| Byte | Value | Meaning |
|---|---|---|
| 0 | `FF` | header |
| 1 | `00` | header |
| 2 | `55` or `AA` | type: `55` = event, `AA` = acquisition points |
| 3 | len | payload length, 0–255 |
| 4… | payload | |
| last | CRC-8 | polynomial 0x07, initial value 0, over type + len + payload |

Multi-byte values are little endian. Example: "run motor at 100.00 Hz" = `FF 00 55 03 05 10 27 4F`.

## Acquisition points (type `AA`)

The payload is a whole number of 10-byte points (the firmware sends up to 20 per packet):

| Bytes | Type | Meaning |
|---|---|---|
| 0–3 | u32 | timestamp in µs (wraps every 71.6 min) |
| 4–6 | i24 | force, raw ADC counts |
| 7–9 | i24 | position, raw ADC counts, interpolated at the force sample's timestamp |

The PC converts counts to N and mm with its calibration.

## Events (type `55`)

The first payload byte is the event id.

### PC → ESP32

| Id | Name | Arguments | Notes |
|---|---|---|---|
| `01` | HEARTBEAT | – | sent every 100 ms; any valid packet also counts |
| `02` | ACQ_START | – | re-synchronises both ADCs and starts streaming |
| `03` | ACQ_STOP | – | also stops the motor |
| `04` | SET_FORCE_LIMITS | i32 lo, i32 hi | raw counts; the motor refuses to run until these are set |
| `05` | MOTOR_RUN | u16 frequency (0.01 Hz) | starts the motor |
| `06` | MOTOR_SET_FREQ | u16 frequency (0.01 Hz) | only while running |
| `07` | MOTOR_STOP | – | |
| `08` | CLEAR_FAULT | – | clears the latched fault and sends a fault reset to the drive |

Every command except HEARTBEAT is answered with an ACK.

### ESP32 → PC

| Id | Name | Payload after the id |
|---|---|---|
| `81` | STATUS (every 100 ms) | u8 state, u8 fault, u8 flags, u16 drive output frequency (0.01 Hz), u16 drive fault code, u16 dropped samples, u8 drive status word |
| `82` | ACK | u8 event id, u8 result |

- **state**: 0 idle, 1 running, 2 fault (latched until CLEAR_FAULT)
- **fault**: 0 none, 1 PC heartbeat lost (> 500 ms), 2 force limit, 3 safety circuit open, 4 drive communication lost, 5 drive fault, 6 ADC samples stopped
- **flags**: bit0 safety circuit closed, bit1 acquisition on, bit2 force limits set, bit3 drive answering, bit4 ADCs running
- **result**: 0 OK, 1 bad arguments, 2 in fault, 3 safety open, 4 no force limits, 5 acquisition off, 6 ADC not ready, 7 not running, 8 unknown command
