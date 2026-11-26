// DynamiX shock dyno - hardware and tuning configuration
#pragma once
#include <stdint.h>

// ---------- PC link (USB serial) ----------
#define PC_BAUD               460800

// ---------- ADS1256 #1 : force (SPI bus 1 / VSPI) ----------
#define PIN_ADC1_SCK          18
#define PIN_ADC1_MISO         19
#define PIN_ADC1_MOSI         23
#define PIN_ADC1_CS           25
#define PIN_ADC1_DRDY         5
#define PIN_ADC1_SYNC         27
#define PIN_ADC1_RST          26

// ---------- ADS1256 #2 : position (SPI bus 2 / HSPI) ----------
#define PIN_ADC2_SCK          14
#define PIN_ADC2_MISO         34
#define PIN_ADC2_MOSI         13
#define PIN_ADC2_CS           33
#define PIN_ADC2_DRDY         32
#define PIN_ADC2_SYNC         22
#define PIN_ADC2_RST          21

#define ADC_SPI_HZ            1000000   // must stay below 1.9 MHz (fCLKIN / 4)
#define ADC_DRATE             0x92      // 500 SPS  (0xA1 = 1000 SPS, 0x82 = 100 SPS)
#define ADC1_ADCON            0x06      // force:    PGA 64, clock out off
#define ADC2_ADCON            0x00      // position: PGA 1,  clock out off
#define ADC_MUX               0x01      // AIN0 (+) / AIN1 (-)  = board terminals AIN1 / AIN2
#define ADC_STATUS            0x02      // two-board mode: MSB first, auto-calibration OFF, input buffer ON.
// STATUS bits: 3 = ORDER, 2 = ACAL (auto-calibration), 1 = BUFEN (input buffer). With 0x06
// (auto-calibration on) the force read about 40x too small; 0x02 fixed it (2026-11-17).
// (Earlier comments said 0x02 turned the buffer off: it is auto-calibration that 0x02 turns off.)

// ---------- single-board mode: load cell AND LVIT on ADS #1 ----------
// The second board disturbed the force reading (about 6-8 N instead of 1 N). With SINGLE_ADC,
// ADS #1 alternates a force conversion (AIN0/AIN1, gain 64) and a position conversion (AIN2/AIN3,
// gain 1), restarting the converter (SYNC + WAKEUP) after each switch so every result is settled.
// ADS #2 is not used (unplug it). Validated with firmware/adc_test (SINGLE_BOARD) on 2026-11-19.
//   load cell  on chip AIN0 (+) / AIN1 (-)  = board terminals AIN1 / AIN2
//   LVIT pair  on chip AIN2 (+) / AIN3 (-)  = board terminals AIN3 / AIN4
#define SINGLE_ADC            1
#define ADC_SINGLE_STATUS     0x00      // MSB first, auto-calibration OFF (it would recalibrate at every gain change), buffer off
#define ADC_SINGLE_DRATE      0xB0      // 2000 conversions/s -> about 600 force + position pairs/s
#define ADC_MUX_FORCE         0x01      // AIN0 (+) / AIN1 (-)
#define ADC_MUX_POSITION      0x23      // AIN2 (+) / AIN3 (-)
#define ADC_CONVERSION_TIMEOUT_US 5000  // one conversion takes about 0.7 ms

// ---------- MAX485 / VFD (Modbus RTU on UART2) ----------
#define PIN_RS485_TX          17
#define PIN_RS485_RX          16
#define PIN_RS485_DE          4
#define VFD_BAUD              9600      // as tested in the original DynamiX code (TD350 P14)
#define VFD_SERIAL_CONFIG     SERIAL_8E1
#define VFD_ADDRESS           3
#define VFD_TIMEOUT_MS        150
#define VFD_POLL_MS           200       // drive status polling interval. The drive's communication timeout must be longer.

// TD350 registers. 0x2000 run/stop, 0x800A and 0x3000 are tested on the drive (original DynamiX code).
// 0x2100 status, 0x2102 fault and the fault-reset command 7 follow the INVT GD350 convention and are NOT
// tested yet; if the drive rejects them the firmware stops asking and carries on without them.
#define VFD_REG_CONTROL       0x2000
#define VFD_REG_FREQ_SET      0x800A    // P00.10 set frequency, 0.01 Hz; 0x8000 = RAM only (no EEPROM wear). Tested.
#define VFD_REG_STATUS        0x2100
#define VFD_REG_FAULT         0x2102
#define VFD_REG_FREQ_OUT      0x3000    // running frequency, 0.01 Hz units. Tested.
#define VFD_CMD_RUN_FWD       0x0001
#define VFD_CMD_STOP          0x0005
#define VFD_CMD_FAULT_RESET   0x0007
#define VFD_STATUS_FAULT      0x0004
#define VFD_MAX_FREQ_CHZ      7000      // refuse commands above 70.00 Hz

// ---------- MLX90614 (GY-906) IR thermometer: damper temperature ----------
// I2C, read only when the PC asks (start of a test and of every speed, and about once a second while
// no test runs), so the I2C traffic never runs alongside a measurement.
//   VIN -> 3.3 V   GND -> GND   SCL -> GPIO 0   SDA -> GPIO 15   (tested with firmware/ir_test)
// GPIO 0 and 15 are boot strapping pins: the module's pull-ups keep them high, which is what boot needs.
#define USE_IR                1         // 0 = no sensor fitted (the PC then shows no temperature)
#define PIN_IR_SDA            15
#define PIN_IR_SCL            0
#define IR_I2C_HZ             50000     // the MLX90614 allows up to 100 kHz
#define IR_ADDRESS            0x5A      // factory address

// ---------- Safety circuit state input ----------
// Dry (potential-free) relay contact from the drive, between GPIO35 and GND.
// External 1 k pull-up to 3V3 and 100 nF to GND at the pin (GPIO35 has no internal pull-up).
// Use the contact that is CLOSED when the safety circuit is armed: an open contact
// or a broken wire then reads HIGH and stops the motor.
#define PIN_SAFETY            35
#define SAFETY_OK_LEVEL       LOW       // level read when the safety circuit is closed (armed)
#define SAFETY_DEBOUNCE       3         // consecutive "open" readings (20 ms apart) before tripping

// ---------- Supervision ----------
#define HEARTBEAT_TIMEOUT_MS  500       // motor stops if the PC is silent this long
// PC silent this long (application closed, crashed or frozen, cable unplugged): after the stop is confirmed the
// ESP32 stops talking to the drive altogether, so the drive's own communication timeout (P14 group) trips it into
// a fault, as when the ESP32 loses power. Communication resumes when the application is back; clear the drive
// fault with "Clear fault". Longer than HEARTBEAT_TIMEOUT_MS so a short UI freeze while idle does not trip the drive.
#define PC_LOST_DRIVE_OFF_MS  3000
// Single-board mode: DRDY found low by polling (interrupt missed) only counts after this long (us).
#define ADC_MISSED_EDGE_US    1500
#define ADC_TIMEOUT_MS        100       // motor stops if samples stop arriving
#define FORCE_LIMIT_SAMPLES   3         // consecutive samples beyond the limit before tripping
#define VFD_COMM_FAIL_LIMIT   5         // consecutive Modbus failures before "VFD comm lost"

// ---------- Streaming ----------
#define POINTS_PER_PACKET     20        // 20 x 10 bytes = 200-byte payload
#define PACKET_FLUSH_MS       20
#define STATUS_PERIOD_MS      100
#define SAMPLE_QUEUE_LEN      512
