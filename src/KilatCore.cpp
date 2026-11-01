/*
 * KilatCore.cpp
 * =============================================================================
 * Hardware-independent logic (see KilatCore.h).  No Arduino headers.
 * =============================================================================
 */
#include "KilatCore.h"

namespace kilat {

/* ========================================================================== *
 *  Robot profiles
 * ========================================================================== *
 *  Field order follows struct Profile in KilatCore.h.
 * ========================================================================== */
const Profile kHammerHead11 = {
  11, 0x07FF, 5, 1,        /* irCount, irMask, irCenter, irOnDefault        */
  true, true,              /* hasTurnType3, hasLineAggregation              */
  3, 3,                    /* turnT, turnSide                               */
  1, 2,                    /* spdUpMs, spdUpStep                            */
  50, 150, 50,             /* spdLo, spdHi, spdTargetMin                    */
  75, 100, 100,            /* actSpdT1, T2, T3                              */
  100, 200,                /* traceLo, traceHi                              */
  25,                      /* actSpdLo                                      */
  7, 8, 2, 3,              /* left lo/hi bit, right lo/hi bit               */
  true, true, true,        /* yThree, tTwo, limitTraceSpeed                 */
  THRESHOLD_MIN_X3
};

const Profile kHammerHead5 = {
  5, 0x001F, 2, 0,
  true, false,
  3, 3,
  2, 1,
  25, 100, 25,
  75, 100, 100,
  100, 200,
  25,
  3, 4, 0, 1,
  false, false, false,
  THRESHOLD_MIN_X3
};

const Profile kComel2 = {
  5, 0x001F, 2, 0,
  false, false,
  1, 2,
  2, 1,
  25, 100, 25,
  60, 75, 0,
  50, 100,
  50,
  3, 4, 0, 1,
  true, true, false,
  THRESHOLD_MID_X1_04
};

const Profile kComel3 = {
  5, 0x001F, 2, 0,
  false, false,
  1, 2,
  2, 1,
  25, 100, 25,
  60, 75, 0,
  50, 100,
  50,
  3, 4, 0, 1,
  true, true, false,
  THRESHOLD_MIN_X3
};

/* ========================================================================== *
 *  Small helpers
 * ========================================================================== */
long arduinoMap(long x, long inMin, long inMax, long outMin, long outMax) {
  /* Classic ArduinoCore-avr formula (integer division, no clamping). */
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

static inline long clampL(long v, long lo, long hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

static inline bool bitOf(uint16_t v, uint8_t i) { return ((v >> i) & 1u) != 0u; }

/* ========================================================================== *
 *  Sensors
 * ========================================================================== */
uint16_t irWord(const Profile &p, const uint8_t *values,
                const uint8_t *thresholds, bool lineColor) {
  uint16_t word = 0;
  for (uint8_t i = 0; i < p.irCount; i++) {
    const bool on = lineColor ? (values[i] < thresholds[i])
                              : (values[i] > thresholds[i]);
    if (on) word = (uint16_t)(word | (1u << i));
  }
  return word;
}

uint8_t aggregatePattern(uint16_t ir) {
  uint8_t out = 0;
  if (bitOf(ir, 10) || bitOf(ir, 9))                    out |= 1u << 4; /* left_2  */
  if (bitOf(ir, 8)  || bitOf(ir, 7))                    out |= 1u << 3; /* left_1  */
  if (bitOf(ir, 6)  || bitOf(ir, 5) || bitOf(ir, 4))    out |= 1u << 2; /* mid     */
  if (bitOf(ir, 3)  || bitOf(ir, 2))                    out |= 1u << 1; /* right_1 */
  if (bitOf(ir, 1)  || bitOf(ir, 0))                    out |= 1u << 0; /* right_2 */
  return out;
}

uint8_t junctionPattern(const Profile &p, uint16_t ir) {
  return p.hasLineAggregation ? aggregatePattern(ir) : (uint8_t)(ir & 0xFFu);
}

int16_t computeError(const Profile &p, uint16_t ir) {
  int16_t error = 0;
  int16_t total = 0;
  for (uint8_t i = 0; i < p.irCount; i++) {
    if (bitOf(ir, i)) {
      error = (int16_t)(error + ((int16_t)i - (int16_t)p.irCenter) * -100);
      total++;
    }
  }
  return (total > 0) ? (int16_t)(error / total) : (int16_t)0;
}

uint8_t updateLastIR(const Profile &p, uint16_t ir, uint8_t lastIR) {
  bool leftBit, rightBit;
  if (p.hasLineAggregation) {
    const uint8_t pat = aggregatePattern(ir);
    leftBit  = ((pat >> 4) & 1u) != 0u;
    rightBit = (pat & 1u) != 0u;
  } else {
    leftBit  = bitOf(ir, (uint8_t)(p.irCount - 1));
    rightBit = bitOf(ir, 0);
  }
  if (leftBit && !rightBit)  return 0b10;
  if (!leftBit && rightBit)  return 0b01;
  return lastIR;
}

/* ========================================================================== *
 *  Junction patterns
 * ========================================================================== */
bool isYJunction(const Profile &p, uint8_t v) {
  const bool a = (v & 0b10010) == 0b10010;
  const bool b = (v & 0b01001) == 0b01001;
  const bool c = (v & 0b10001) == 0b10001;
  return a || b || (p.yThreePatterns && c);
}

bool isTJunction(const Profile &p, uint8_t v) {
  const bool a = (v & 0b10101) == 0b10101;
  const bool b = (v & 0b01110) == 0b01110;
  return a || (p.tTwoPatterns && b);
}

bool isLeftJunction(uint8_t v)  { return (v & 0b10101) == 0b10100; }
bool isRightJunction(uint8_t v) { return (v & 0b10101) == 0b00101; }

/* ========================================================================== *
 *  PID (PD)
 * ========================================================================== */
MotorPair pidStep(PidState &s, uint8_t spd, int16_t error, float kp, float kd,
                  uint32_t nowMs) {
  const int32_t dt = (int32_t)(nowMs - s.prevTimeMs);
  const float derivative =
      (dt > 0) ? ((float)((int32_t)error - s.prevError) / (float)dt) : 0.0f;

  float control = (kp * (float)error) + (kd * derivative);
  if (control != control)  control = 0.0f;       /* NaN guard              */
  if (control >  30000.0f) control =  30000.0f;  /* keep the cast defined  */
  if (control < -30000.0f) control = -30000.0f;

  s.prevError  = error;
  s.prevTimeMs = nowMs;

  const int16_t c = (int16_t)control;            /* truncates toward zero  */
  const int16_t left  = (int16_t)((int16_t)spd + c);
  const int16_t right = (int16_t)((int16_t)spd - c);

  MotorPair m;
  m.left  = (int16_t)clampL(left,  -(long)spd, (long)spd);
  m.right = (int16_t)clampL(right, -(long)spd, (long)spd);
  return m;
}

/* ========================================================================== *
 *  Junction-following state machine
 * ========================================================================== */
void junctionBegin(const Profile &p, JunctionState &st, uint16_t spd,
                   uint32_t nowMs, uint8_t lastIR, uint32_t timerSpdUp) {
  st.spdTarget  = (uint8_t)clampL(spd, p.spdTargetMin, 255);
  st.spd        = (uint16_t)clampL(spd, p.spdLo, p.spdHi);
  st.timerSpdUp = timerSpdUp;
  st.enterTime  = nowMs;
  st.lastIR     = lastIR;
  st.turnType   = 0;
}

StepResult junctionStep(const Profile &p, JunctionState &st, char type,
                        int16_t skipDuration, bool gap, uint16_t ir,
                        uint32_t nowMs) {
  StepResult r;
  r.kind = StepKind::Follow;
  r.error = 0;
  r.spd = st.spd;
  r.traceSpd = 0;
  r.traceLeft = false;
  r.driveForward = false;

  /* ---- Robot lost the line ---- */
  if (ir == 0) {
    r.kind = StepKind::Lost;
    if (!gap) {
      r.traceSpd  = (uint8_t)arduinoMap(st.spd, 50, 255, p.traceLo, p.traceHi);
      r.traceLeft = (st.lastIR == 0b10);
      if (p.limitTraceSpeed && st.spd > 150) st.spd = 150;
    } else {
      r.driveForward = true;
      r.spd = st.spd;
    }
    return r;
  }

  /* ---- At least one sensor on the line: speed ramp ---- */
  if (st.spd < st.spdTarget) {
    if ((uint32_t)(nowMs - st.timerSpdUp) >= p.spdUpMs) {
      st.timerSpdUp = nowMs;
      const uint16_t next = (uint16_t)(st.spd + p.spdUpStep);
      st.spd = (next > st.spdTarget) ? st.spdTarget : next;  /* never overshoot */
    }
  } else {
    st.spd = st.spdTarget;
  }

  const uint8_t pattern = junctionPattern(p, ir);

  if (type == 'Y' && isYJunction(p, pattern)) {
    st.turnType = 2;
    r.kind = StepKind::Detected;
    return r;
  }
  if (type == 'T' && isTJunction(p, pattern)) {
    st.turnType = p.turnT;
    r.kind = StepKind::Detected;
    return r;
  }
  if (type == 'L' && isLeftJunction(pattern)) {
    st.turnType = p.turnSide;
    r.kind = StepKind::Detected;
    return r;
  }
  if (type == 'R' && isRightJunction(pattern)) {
    st.turnType = p.turnSide;
    r.kind = StepKind::Detected;
    return r;
  }
  /* SKIP: end after skipDuration ms.  skipDuration == -1 (or any negative
   * value) converts to a huge unsigned number, so such a junction never ends
   * and the robot simply keeps following the line.  (The original code had an
   * extra "if (skipDuration != -1)" inside this branch; it was unreachable, so
   * it was removed - see CHANGELOG.) */
  if (type == 'S' && (uint32_t)(nowMs - st.enterTime) >= (uint32_t)skipDuration) {
    st.turnType = 1;
    r.kind = StepKind::Detected;
    return r;
  }

  r.kind  = StepKind::Follow;
  r.error = computeError(p, ir);
  r.spd   = st.spd;

  st.lastIR = updateLastIR(p, ir, st.lastIR);
  return r;
}

/* ========================================================================== *
 *  Turning at a junction
 * ========================================================================== */
TurnPlan planTurn(const Profile &p, uint8_t turnType, char dir,
                  uint8_t requestedSpd) {
  TurnPlan t;
  t.valid = false; t.move = false; t.right = false;
  t.spdL = 0; t.spdR = 0; t.settleMs = 0;

  const bool left  = (dir == 'L');
  const bool right = (dir == 'R');
  if (!left && !right) return t;

  t.valid = true;
  t.right = right;

  uint8_t spd = requestedSpd;
  if (turnType == 1) {
    spd = (requestedSpd == 0) ? p.actSpdT1
                              : (uint8_t)clampL(requestedSpd, p.actSpdLo, 150);
    t.move = true; t.spdL = spd; t.spdR = spd;
  } else if (turnType == 2) {
    spd = (requestedSpd == 0) ? p.actSpdT2
                              : (uint8_t)clampL(requestedSpd, p.actSpdLo, 150);
    const uint16_t slow = (uint16_t)arduinoMap(spd, 25, 150, 0, 31);
    t.move = true;
    t.spdL = right ? spd : slow;
    t.spdR = right ? slow : spd;
  } else if (p.hasTurnType3 && turnType == 3) {
    spd = (requestedSpd == 0) ? p.actSpdT3
                              : (uint8_t)clampL(requestedSpd, p.actSpdLo, 150);
    const uint16_t slow = (uint16_t)arduinoMap(spd, 25, 150, 0, 63);
    t.move = true;
    t.spdL = right ? spd : slow;
    t.spdR = right ? slow : spd;
  }

  const long settle = arduinoMap(spd, p.actSpdLo, 150, 150, 50);
  t.settleMs = (uint16_t)(settle < 0 ? 0 : settle);
  return t;
}

bool turnReacquired(const Profile &p, bool turnLeft, uint16_t ir) {
  if (turnLeft)
    return bitOf(ir, p.turnLeftLoBit) || bitOf(ir, p.turnLeftHiBit);
  return bitOf(ir, p.turnRightLoBit) || bitOf(ir, p.turnRightHiBit);
}

/* ========================================================================== *
 *  Calibration, buttons, battery, display
 * ========================================================================== */
uint8_t thresholdFromRange(const Profile &p, uint8_t minV, uint8_t maxV) {
  float t;
  if (p.thresholdMode == THRESHOLD_MID_X1_04)
    t = (float)(((uint16_t)minV + maxV) / 2) * 1.04f;
  else
    t = (float)minV * 3.0f;
  if (t > 255.0f) t = 255.0f;   /* saturate: the original overflowed */
  return (uint8_t)t;
}

uint8_t decodeButtons(uint8_t upDownRaw, uint8_t startRaw) {
  uint8_t state = 0;
  if (startRaw < 100)                          state |= 0b100;  /* START */
  if (upDownRaw < 100)                         state |= 0b001;  /* DOWN  */
  if (upDownRaw > 100 && upDownRaw < 200)      state |= 0b010;  /* UP    */
  return state;
}

uint8_t batteryPercent(uint8_t raw) {
  const long v = clampL(raw, 128, 150);
  return (uint8_t)((v - 128) * 100 / (150 - 128));
}

uint8_t batteryLevel(uint8_t percent) {
  return (uint8_t)arduinoMap(percent, 0, 100, 0, 14);
}

uint8_t formatElapsed(uint32_t ms, char *out, uint8_t capacity) {
  /* "S.CCsec": seconds, a dot, two-digit hundredths, then "sec". */
  char tmp[16];
  uint8_t n = 0;

  uint32_t secs = ms / 1000u;
  const uint32_t hundredths = (ms % 1000u) / 10u;

  char digits[10];
  uint8_t d = 0;
  do { digits[d++] = (char)('0' + (secs % 10u)); secs /= 10u; } while (secs > 0 && d < 10);
  while (d > 0) tmp[n++] = digits[--d];

  tmp[n++] = '.';
  tmp[n++] = (char)('0' + (hundredths / 10u));
  tmp[n++] = (char)('0' + (hundredths % 10u));
  tmp[n++] = 's'; tmp[n++] = 'e'; tmp[n++] = 'c';

  if (capacity == 0) return 0;
  uint8_t w = 0;
  while (w < n && w + 1 < capacity) { out[w] = tmp[w]; w++; }
  out[w] = '\0';
  return w;
}

}  /* namespace kilat */
