# Migrating from the earlier versions

| Before | Now |
| --- | --- |
| six `.h` files, or `kilat.h` + `kilat.cpp` copied into the sketch folder | install the library once (`KilatTracer`), then `#include <KilatTracer.h>` |
| `#include "kilat.h"` | still works (compatibility header); prefer `<KilatTracer.h>` |
| robot type `#define`d in the sketch | choose it in `src/KilatConfig.h` or with `-DHammerHead_11` (sketch `#define`s were never visible to the library) |
| `#include <U8x8lib/U8x8lib.h>` | `<U8x8lib.h>` from the U8g2 library (install "U8g2") |
| `library.properties`: name `StemBot`, `includes=StemBot2019.h`, `architectures=samd` | name `KilatTracer`, `includes=KilatTracer.h`, `architectures=avr` |

The public functions and globals are unchanged. mBlock-generated code that
includes `kilat.h` keeps working; check `js/` and `StemBotKilat.s2e` if you
regenerate it.

Behaviour changes (all bug fixes) are listed in [CHANGELOG.md](../CHANGELOG.md).
