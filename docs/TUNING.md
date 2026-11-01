# Tuning guide

The controller is a PD controller on the line position:

```
control = KP * error + KD * (change of error per millisecond)
left motor  = spd + control      right motor = spd - control   (both limited to +-spd)
```

`error` is 0 when centred, grows by 100 per sensor step, and is positive when
the line is to the right. `KD` therefore multiplies a quantity in error units per
millisecond.

Starting values used by the example sketch: `spd = 100`, `KP = 0.4`, `KD = 0.2`.
These are a starting point for the robot the example was written for, not a
universal setting; your track, tyres and battery will differ.

## Procedure
1. **Calibrate** (hold UP at power-on) on the real track, forward and back over
   the line, with a charged battery. Check the bars on the OLED: the sensor over
   the line should show a clearly higher bar than over the background.
2. **Slow first.** Set a low `spd` and `KD = 0`. Raise `KP` until the robot
   follows the line; if it wobbles, lower `KP`.
3. **Add `KD`** to damp the wobble. Increase it until the oscillation on
   curves disappears; too much makes the motion jittery.
4. **Raise `spd`** step by step. Each time, re-check `KP` and `KD`; the speed
   ramp (`spdUpStep` per `spdUpMs`) lets the robot accelerate onto the line.
5. **Junctions:** if a junction is missed, lower the speed approaching it; if
   the robot overshoots a turn, use a lower `action()` speed.
6. **Lost line:** `gap = false` traces back toward the side the line was last
   seen; use `gap = true` only on tracks with real gaps.

## Per-robot limits
`spdLo`, `spdHi`, turn speeds and trace-back speeds are in `kilat::Profile`
(`src/KilatCore.cpp`). The library clamps the speeds you pass to these limits.
