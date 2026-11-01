#!/bin/sh
# Type-checks the hardware layer for every robot type and optional feature
# against the stub Arduino API (no hardware, no AVR toolchain needed).
# Real-board builds are done by CI with arduino-cli (see .github/workflows).
set -e
cd "$(dirname "$0")/.."
CXX=${CXX:-g++}
FLAGS="-std=gnu++11 -Wall -Wextra -Wshadow -Werror -fsyntax-only -Itest/stubs -Isrc"
fail=0
for robot in HammerHead_5 HammerHead_11 Comel2 Comel3 NONE; do
  for feat in "" "-Doled_OFF" "-DBT_ON -Dcamera_ON" "-DHusky_ON" "-DServo1_OFF -DServo2_OFF" "-Doled_OFF -DBT_ON -DHusky_ON -Dcamera_ON"; do
    r=""; [ "$robot" != "NONE" ] && r="-D$robot"
    if $CXX $FLAGS $r $feat src/KilatTracer.cpp 2>/tmp/kilat_cc.log; then
      echo "ok    ${robot} ${feat}"
    else
      echo "FAIL  ${robot} ${feat}"; cat /tmp/kilat_cc.log; fail=1
    fi
  done
done
# the example sketch must compile for every robot type
for robot in HammerHead_5 HammerHead_11 Comel2 Comel3; do
  if $CXX -std=gnu++11 -fsyntax-only -Itest/stubs -Isrc -D$robot -x c++ -include Arduino.h examples/JunctionCourse/JunctionCourse.ino 2>/tmp/kilat_cc.log; then
    echo "ok    example sketch ($robot)"
  else
    echo "FAIL  example sketch ($robot)"; cat /tmp/kilat_cc.log; fail=1
  fi
done
# two robot types at once must be rejected
if $CXX $FLAGS -DHammerHead_5 -DComel2 src/KilatTracer.cpp 2>/dev/null; then
  echo "FAIL  conflicting robot types were accepted"; fail=1
else
  echo "ok    conflicting robot types are rejected"
fi
exit $fail
