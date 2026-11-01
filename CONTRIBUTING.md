# Contributing

## Build and test
```sh
make test        # needs only g++ and make
```
Run it before every commit; it includes `make diff`, which compares the code with the
original firmware. If you change behaviour on purpose, say so in the changelog and
adjust the scenarios that the change affects. It builds with `-Wall -Wextra -Wshadow -Werror` and
AddressSanitizer / UndefinedBehaviorSanitizer.

## Rules of thumb
* Decisions go in `src/KilatCore.*` (no Arduino headers there); the hardware
  layer only reads and writes pins.
* Every change to logic comes with a test. Tests for a bug fix are marked `FIX`.
* Keep the public API stable; note changes in `CHANGELOG.md`.
* No `String`, no `malloc`/`new` in library code (small RAM).
* Test on a robot before tagging a release.

## Commits and releases
Small commits with a clear subject line. Releases are tagged `vMAJOR.MINOR.PATCH`
and `library.properties` and `CHANGELOG.md` are updated in the same commit.

## Reporting a bug
Use the bug-report template: robot type, library version, what you expected,
what happened, and the sketch.
