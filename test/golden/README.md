# Golden reference: the original firmware

`legacy/KilatTracer.h` and `legacy/KilatTracer.cpp` are the **original** library
files (the version before the refactor), kept only as a reference for the
differential test (`make diff`).

Two test-only patches were applied so they compile for every robot type; both
are marked `PATCH` in the header and neither changes behaviour:

1. the default `#define HammerHead_5` is skipped when another robot is selected;
2. the U8g2 include path is `<U8x8lib.h>`.

Do not use these files in a sketch.
