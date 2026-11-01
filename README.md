# Kilat Line Tracer Robot

[![CI](https://github.com/MokRH/Kilat-LineTracerRobot/actions/workflows/ci.yml/badge.svg)](https://github.com/MokRH/Kilat-LineTracerRobot/actions/workflows/ci.yml)

Arduino firmware library for the **KILAT** line-following robots (HammerHead 5,
HammerHead 11, Comel 2, Comel 3) built on the ATmega328P. It provides PD line
following, junction detection (Y, T, left, right, skip), IR calibration, an OLED
status display, and optional Bluetooth, camera, HUSKYLENS and gripper support.

The robots are used in school STEM programmes and competitions, and in the
*UMP STEM Bot* educational kit (see [Citation](#citation)). A custom mBlock
extension (`js/`, `StemBotKilat.s2e`) lets learners program them with blocks.

**Design goal of version 2:** all decisions (sensor word, line position,
junction detection, PD control, turn planning, thresholds) live in a
hardware-independent core that is unit-tested on a PC; the hardware layer only
reads pins and drives outputs. See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

## Supported robots

| Robot (`#define`) | IR sensors | Notes |
| --- | --- | --- |
| `HammerHead_5`  | 5  | default if nothing is selected |
| `HammerHead_11` | 11 | folded into a 5-bit pattern for junction detection |
| `Comel2`        | 5  | threshold = midpoint x 1.04 |
| `Comel3`        | 5  | threshold = 3 x minimum |

## Quick start

1. Install the library (Sketch > Include Library > Add .ZIP Library, or copy
   this folder into your Arduino `libraries` folder) and the **U8g2** library
   (needed for the OLED, unless you build with `oled_OFF`).
2. Choose the robot, either by editing `src/KilatConfig.h` or with a build flag:

   ```sh
   arduino-cli compile --fqbn arduino:avr:nano:cpu=atmega328 \
     --build-property "build.extra_flags=-DHammerHead_11" examples/JunctionCourse
   ```

   `#define`s in your sketch are **not** seen by the library, because it is
   compiled as a separate translation unit.
3. Write your sketch:

   ```cpp
   #include <KilatTracer.h>

   void setup() { KILAT_setup(); }          // pins, PWM, OLED, calibration, wait for START
   void loop() {
     junction("Y", 100, 0.4, 0.2, 0, false, false);   // follow the line until a Y junction
     action("RIGHT", 0, 0, false);                    // turn right until the line is found
   }
   ```

   Hold **UP** while switching on to calibrate the IR sensors; press **START**
   to run. The full example is in [`examples/JunctionCourse`](examples/JunctionCourse).

## Documentation

| Document | Contents |
| --- | --- |
| [docs/API.md](docs/API.md) | every function, global and option |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | layers, data flow, junction state machine, design decisions |
| [docs/TUNING.md](docs/TUNING.md) | speed, KP, KD and calibration |
| [docs/ADDING_A_ROBOT.md](docs/ADDING_A_ROBOT.md) | how to add a new robot profile |
| [docs/MIGRATION.md](docs/MIGRATION.md) | moving from the single-file `kilat.h`/`kilat.cpp` version |
| [docs/KNOWN_ISSUES.md](docs/KNOWN_ISSUES.md) | limitations and things not yet verified on hardware |
| [CHANGELOG.md](CHANGELOG.md) | what changed, including bug fixes |
| [CONTRIBUTING.md](CONTRIBUTING.md) | how to build, test and propose changes |

## Testing

No robot and no AVR toolchain are needed for the host tests, only `g++` and `make`:

```sh
make test      # everything below
make core      # only the hardware-independent unit tests (with sanitizers)
make diff      # differential test: original firmware vs the new code
```

* `test/test_core.cpp` pins down the behaviour of the logic (profiles, sensor
  word, line error, junction patterns, PD step, junction state machine, turn
  planning, thresholds, buttons, battery, time formatting).
* `test/test_hal.cpp` runs the real hardware layer against a stub Arduino API
  with a scripted virtual line sensor, once per robot type.
* `test/compile_matrix.sh` type-checks the library for every robot and every
  optional feature, and the example for every robot.
* `make diff` (differential test) runs the **original firmware**
  (`test/golden/legacy`) and the new code through the same 1500 random junction
  scenarios, 1500 turn scenarios, PD, button, IR-read and calibration runs per
  robot, and requires the traces (every PWM/LED write with its timestamp, plus
  the resulting state) to be identical. The test was also checked to fail when
  the logic is deliberately broken (mutation checks).
* GitHub Actions also compiles the example for each robot with `arduino-cli`
  (see `.github/workflows/ci.yml`).

The tests cover the logic and the glue. They do **not** replace a run on the
robot: see [docs/KNOWN_ISSUES.md](docs/KNOWN_ISSUES.md).

## Installing the custom mBlock

1. Download the latest custom mBlock at <https://drive.google.com/file/d/14SeQ3soiTEHgXAxd6-Lw7MC8B_yi80yt/view?usp=sharing>
2. Find the downloaded `.exe` file and double-click to start the installation.
3. Change the installation location to `C:\` and start the installation by clicking *Extract*.
4. When the installation has finished, open the `C:\mBlock_MokRH` folder, copy
   `mBlock.exe` and paste a shortcut on the desktop.
5. Done, and have fun.

Thank you for supporting Stembot-2019 and the Kilat Comel robotic kit.

## Citation

The educational kit these robots belong to is described in: N. H. Noordin,
M. R. Hao (listed as Mok Ren Hau), I. H. Mohamad, *UMP STEM Bot as Educational
Kit for Robotics and Computer Programming*, 8th International Conference on
Mechatronics Engineering (ICOM 2022), 2022. DOI: 10.1049/icp.2022.2256.

## Licence

MIT, see [LICENSE](LICENSE).
