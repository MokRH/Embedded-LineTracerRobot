# Changelog

Format: [Keep a Changelog](https://keepachangelog.com/), versions follow
[Semantic Versioning](https://semver.org/).

## [2.0.0] - 2026-11-01

### Added
- `KilatCore.h/.cpp`: the decision logic as pure functions with no Arduino
  dependency (robot profiles, sensor word, line error, junction patterns, PD
  step, junction state machine, turn planning, thresholds, buttons, battery,
  time formatting).
- Differential test (`make diff`) against the original firmware kept in
  `test/golden/legacy`: identical traces for all four robots.
- Unit tests (`test/test_core.cpp`) and hardware-layer tests against a stub
  Arduino API with a scripted virtual line sensor (`test/test_hal.cpp`), a
  compile matrix over every robot and option, `make test`, and GitHub Actions
  CI (host tests, `arduino-cli` builds for each robot, `arduino-lint`).
- `KilatConfig.h`: one place to choose the robot and options; build flags
  (`-DHammerHead_11`) now work.
- `KILAT_muxSelectByte()` for diagnostics and tests.
- `src/kilat.h` compatibility header so `#include "kilat.h"` keeps working.
- Documentation: README, API reference, architecture, tuning guide, guide to
  adding a robot, migration guide, known issues; `keywords.txt`; `LICENSE`.
- Example sketch moved to `examples/JunctionCourse/JunctionCourse.ino`.

### Changed
- Library renamed to **KilatTracer** (header `KilatTracer.h`). `library.properties`
  now matches the code: `includes=KilatTracer.h`, `architectures=avr` (was
  `samd`), `depends=U8g2`, correct URL, version 2.0.0.
- `junction()`, `action()`, `KILAT_PID()`, `KILAT_IRupdate()`, `KILAT_getButton()`
  and the calibration/OLED helpers now delegate their decisions to the core.
- The Arduino `String` class is no longer used (no heap allocation).

### Fixed
- Selecting a robot with a build flag failed: the header always defined
  `HammerHead_5`, so the "only one robot type" check fired. It now defaults to
  HammerHead_5 only when nothing is selected.
- `#include <U8x8lib/U8x8lib.h>` pointed at a path that does not exist in the
  U8g2 library; it is now `<U8x8lib.h>`.
- The example included `"kilat.h"` while the header was `KilatTracer.h`, and
  defined the robot type in the sketch, where the library cannot see it.
- Speed ramp could overshoot the target (HammerHead_11 steps by 2, so 254 + 2 =
  256, truncated to 0 when passed to the PD controller as `uint8_t`) and stall
  the motors for one cycle. It now saturates at the target.
- IR threshold calibration cast an out-of-range float to `uint8_t` (undefined
  behaviour, and a wrapped value could make a sensor always trigger). It now
  saturates at 255.
- PD controller: the float control value is limited before the integer cast,
  and NaN gains give zero control instead of undefined behaviour.
- README described `kilat.h`/`kilat.cpp` while the files were `KilatTracer.*`.

### Removed
- A dead branch in the SKIP junction (`skipDuration != -1` inside a condition
  that can only be true when it is not -1). Behaviour is unchanged: a SKIP with
  a negative duration never ends, a SKIP with a duration >= 0 ends with
  `turnType = 1`.

### Behaviour that is intentionally unchanged
Sensor word, line error, junction patterns (including the HammerHead_5 Y and T
pattern limits), trace-back, turn speeds, PD formula (`KP*error + KD*d error/dt`
with float `dt`), battery and button decoding, and the on-screen time format.

## Earlier history (from the previous README)
- 2026-10-18: reorganised the library code into a cleaner format (`kilat.h` and `kilat.cpp`).
- 2022-06-06: trace delay added.
- Original library for the STEM Bot / Kilat robots: six `.h` files (`library.properties` version 1.0).
