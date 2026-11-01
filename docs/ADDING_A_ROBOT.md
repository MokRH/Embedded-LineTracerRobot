# Adding a robot

1. **Profile** - in `src/KilatCore.cpp` add a `const Profile kMyRobot = {...}`
   (field order is documented in `src/KilatCore.h`) and declare it `extern` in
   `KilatCore.h`.
2. **Configuration** - in `src/KilatTracer.h` add `MyRobot` to the "only one robot
   type" check and a `#elif defined(MyRobot)` block with `KILAT_PROFILE`,
   `KILAT_IR_COUNT`, `KILAT_IR_MASK`, `KILAT_IR_CENTER`, `KILAT_IR_ON_DEFAULT`.
3. **Hardware** - in `src/KilatTracer.cpp` add the mux pins, the `KILAT_IRnumber`
   order and the mux select table for the new board, and the display name.
4. **Tests** - add the profile to the `kAll` list and write the expectations
   that differ in `test/test_core.cpp`; add the robot to `ROBOTS` in the
   `Makefile`, to `test/compile_matrix.sh` and to the CI matrix.
5. **Docs** - update the table in the README and `docs/API.md`, and add a
   changelog entry.

`make test` must pass before you flash anything.
