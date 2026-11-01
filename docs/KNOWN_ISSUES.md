# Known issues and limits

These come from reading and testing the code. Items marked *(unverified)* have
not been checked on a robot.

## Verified by the host tests
* Trace-identical behaviour with the original firmware on random scenarios
  (`make diff`), for all four robot types.
* Decision logic, state machine and the glue between the multiplexer, ADC,
  thresholds, PD controller and PWM pins, for every robot type.
* The library compiles for every robot and option combination (stub API).

## Not verified on hardware
* Real-board builds: CI compiles the example with `arduino-cli` for each robot
  type, but a first CI run may need small adjustments *(unverified)*.
* Behaviour on the track after the 2.0.0 refactor: flash each robot type you
  own and check calibration, line following and one junction before relying on it.

## Limits of the design
* **Serial is unavailable:** the LEDs are on pins 0 and 1 (the hardware serial
  port), so `Serial` cannot be used for debugging.
* **Blocking calls:** `junction()`, `action()`, `KILAT_waitToStart()`,
  `KILAT_getBT()` and `oledDisplay_time(1)` do not return until their condition
  is met (the last one never returns). `action()` with `duration == 0` loops
  forever if the line is never found. There is no timeout or watchdog *(by design
  in the original firmware; not changed)*.
* **No settling time after switching the multiplexer** before each ADC read;
  readings may be affected by the previous channel *(unverified)*.
* **Grippers** use `analogWrite` on pins 9 and 10 (about 490 Hz), not the Servo
  library; whether a given servo accepts this *(unverified)*.
* **AVR only:** the library writes timer registers directly (`TCCRxx`).
* **Calibration** runs only when UP is held at power-on and the battery reading
  is healthy.
* **`KILAT_getButton()`** includes a 20 ms delay, so polling it slows a loop.
