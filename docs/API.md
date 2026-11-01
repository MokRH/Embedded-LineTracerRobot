# API reference

Include `<KilatTracer.h>` (`"kilat.h"` still works). Choose the robot in
`src/KilatConfig.h` or with a build flag; see the README.

## Options (compile-time)

| Macro | Meaning |
| --- | --- |
| `HammerHead_5`, `HammerHead_11`, `Comel2`, `Comel3` | robot type, define exactly one (default `HammerHead_5`) |
| `BT_ON` | Bluetooth on `SoftwareSerial(7, 8)` |
| `camera_ON` | camera/serial module on `SoftwareSerial(12, 10)` |
| `Husky_ON` | DFRobot HUSKYLENS over I2C (library "HUSKYLENS") |
| `oled_OFF` | build without the OLED |
| `Servo1_OFF`, `Servo2_OFF` | disable the grippers on pins 9 / 10 (on by default) |

## Setup
| Function | Description |
| --- | --- |
| `KILAT_setup()` | configure pins and PWM, OLED, load thresholds, optional peripherals, calibrate if UP is held, wait for START, start the timer |
| `KILAT_waitToStart()` | only the wait-for-START loop (UP / DOWN nudge the robot, START leaves) |

## Line following
| Function | Description |
| --- | --- |
| `junction(type, spd, KP, KD, skip, gap, line_color)` | follow the line until a junction is detected. `type`: `"Y"`, `"T"`, `"LEFT"`, `"RIGHT"`, `"SKIP"` (first letter is used). `spd`: target speed (profile limits apply). `skip`: SKIP duration in ms (negative = never ends). `gap`: `false` traces back to the side the line was last seen, `true` drives straight when the line is lost. `line_color`: `false` black line, `true` white line |
| `action(dir, spd, duration, line_color)` | turn at a junction. `dir`: `"LEFT"` / `"RIGHT"` (anything else stops). `spd` 0 = profile default for the current `turnType`. `duration` 0 = turn until the line is reacquired, otherwise turn for that many ms |
| `KILAT_PID(spd, error, KP, KD)` | one PD step: `control = KP*error + KD*d(error)/dt[ms]`, motors limited to +-`spd` |

## Motors, LEDs, buttons
| Function | Description |
| --- | --- |
| `KILAT_movement(dir, spdL, spdR)` | `"FWD"`, `"REV"`, `"LEFT"`, `"RIGHT"`, `"STOP"`; speeds 0..255; resets `turnType` to 1 |
| `motor_A(spd)`, `motor_B(spd)` | signed speed -255..255 (A = left, B = right) |
| `KILAT_LED(colour, on)` | `"green"`, `"yellow"`, `"red"` (first letter) |
| `KILAT_getButton()` | bit 2 = START, bit 1 = UP, bit 0 = DOWN (with a 20 ms debounce delay) |

## Sensors
| Function | Description |
| --- | --- |
| `KILAT_IRinit()` | load thresholds from EEPROM into `g_IR_threshold[]` |
| `KILAT_IRcalibrate()` | interactive calibration (hold UP at power-on): sweeps forward and back, stores per-sensor thresholds |
| `KILAT_IRupdate(line_color)` | read all sensors into a bit word (bit i = sensor i on the line) |
| `readMux(item)` | 8-bit reading of a logical channel (IR 0..10; `MUX_CH_UP_DOWN`, `MUX_CH_START_BTN`, `MUX_CH_BATT_MON`) |
| `KILAT_muxSelectByte(item)` | select byte of a channel, `0xFF` = unused (diagnostics) |

## OLED
`oledDisplay_setup()`, `oledDisplay_char(col, line, str)`, `oledDisplay_clear()`,
`oledDisplay_showName()`, `oledDisplay_showBatt()`, `oledDisplay_showBar(y, x, percent)`,
`oledDisplay_showIR()`, `oledDisplay_time(0|1)` (0 starts the timer; 1 shows the
finish time and then blinks the LEDs **forever**).

## Optional peripherals
* Bluetooth: `KILAT_btSetup()`, `KILAT_btRename(name)`, `KILAT_getBT()` (blocks until a byte arrives).
* Camera: `KILAT_cameraSetup()`, `KILAT_getCamera(default, timeout_ms)`.
* HUSKYLENS: `KILAT_HuskySetup()` (blocks until the device answers), `KILAT_getHusky()` (ID of the last learned object, 0 if none).
* Grippers: `KILAT_gripper_1(angle, delay_ms)` (pin 9), `KILAT_gripper_2(angle, delay_ms)` (pin 10); PWM pulses of 1-2 ms, not the Servo library.

## Globals
| Name | Meaning |
| --- | --- |
| `turnType` | 1 = turn at centre, 2 = bias to one side (Y), 3 = strong bias (HammerHead) |
| `last_IR` | `0b10` last seen left, `0b01` last seen right |
| `g_IR_threshold[]` | SRAM copy of the thresholds |
| `u8x8`, `btSerial`, `cameraSerial`, `huskylens` | optional peripheral objects |

## Core (hardware-independent) API
`kilat::` functions in `KilatCore.h` (`irWord`, `computeError`, `junctionStep`,
`pidStep`, `planTurn`, `thresholdFromRange`, ...) can be used and tested without
any hardware.
