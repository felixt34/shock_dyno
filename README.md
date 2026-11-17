# DynamiX shock dyno software

| Folder | What it is |
|---|---|
| `firmware/shock_dyno/` | ESP32 firmware (Arduino core for ESP32). Reads both ADS1256, time-aligns force and position, drives the VFD over Modbus RTU, enforces the safety rules. |
| `app/` | Windows application (C#, WinForms, .NET Framework 4.8, no external packages). Runs tests, shows the graphs, stores CSV files. |
| `docs/protocol.md` | Packet protocol between the two. |

## Firmware

1. Arduino IDE with the **esp32** board package. Open `firmware/shock_dyno/shock_dyno.ino`, board "ESP32 Dev Module", upload. (PlatformIO also works: `firmware/platformio.ini`.)
2. Everything hardware-specific is in `config.h`. **Check before the first run:**
   - `VFD_*` — 9600 baud 8E1, slave address 3, run/stop at 0x2000, set frequency at 0x800A (P00.10, RAM) and output frequency at 0x3000, as tested with the original DynamiX code. Status 0x2100, fault 0x2102 and fault reset (0x2000 = 7) are still untested; if the drive rejects them the firmware stops polling them.
   - `PIN_SAFETY` (GPIO35): a dry relay contact from the drive between GPIO35 and GND, with a 1 kΩ pull-up to 3V3 and 100 nF to GND. Use the contact that is closed when the safety circuit is armed (`SAFETY_OK_LEVEL` = LOW), so a broken wire also stops the motor.
   - ADC pins match the proto board (two SPI buses).

The motor only runs when all of these hold; any violation latches a fault and stops the drive:
safety circuit closed · PC heartbeat alive (500 ms) · samples arriving · force inside the limits · drive answering and not in fault.

## Application

Open `app/ShockDyno.sln` in Visual Studio 2022 (".NET desktop development" workload) and run `ShockDyno.App`.

- **Simulator**: choose "Simulator" in the port list to run the whole application without hardware (virtual machine + damper).
- **Tools → Sensor calibration**: zero and gain of the load cell, two-point calibration of the LVIT. Do this once before trusting any number.
- **Tools → Settings**: units (SI / imperial), drive train, minimum and maximum motor frequency, force limit (3000 N), filter, data folder.
- **Sequence**: edit the table directly, or load/save `.seq` text files (`app/sequences/default.seq`).

### What a test does

1. **Calibration cycle** at a low fixed motor frequency (setting `CalibrationMotorHz`, 1 Hz): one settling cycle, one measured cycle.
   - amplitude = half the measured stroke
   - preload = force at bottom dead centre; gas force = static force at mid-stroke. Both are read at the dead centres, where speed and therefore damping are zero.
   - the motor's real speed (slip) is measured and used to correct later frequency commands.
2. **Each step**: motor frequency = speed / (2π · amplitude) × belt ratio × 60 Hz / (1750 rpm / 60). The step waits until two consecutive cycles agree (steady speed), then measures the requested number of cycles. Steps that need a frequency outside the allowed range are skipped with a message.
3. The force-vs-position graph is redrawn after every cycle. Preload and gas force (a straight line between the two dead-centre readings) are removed from the plotted force; their values are shown on the left.
4. The test is saved as one CSV in the data folder (raw calibrated samples + header). Tick tests in the history list to overlay them on the analysis graphs; **File → Export summary CSV** writes peak velocity and force per speed.

Sign convention: + is compression for force, position and velocity.

### Tests

`ShockDyno.Tests` is a console program that runs complete tests against the simulator (three amplitudes, safety trip, force limit, CSV round trip) and checks the results against the simulated damper.
