/*
 * KilatCore.h
 * =============================================================================
 * Hardware-independent logic of the KILAT line-following library.
 *
 * Nothing in this file (or KilatCore.cpp) touches Arduino APIs, pins or timers.
 * Every function is a pure function of its arguments (the few stateful helpers
 * keep their state in a struct that the caller owns).  That is what makes the
 * logic unit-testable on a normal computer (see test/).
 *
 * The behaviour is a faithful extraction of the original firmware logic; the
 * unit tests in test/test_core.cpp pin that behaviour down.  Where the original
 * relied on undefined behaviour (integer/float overflow) the new code saturates
 * instead; those few places are listed in CHANGELOG.md.
 * =============================================================================
 */
#ifndef KILAT_CORE_H
#define KILAT_CORE_H

#include <stdint.h>

namespace kilat {

/* How the per-sensor threshold is derived from the calibration sweep. */
enum ThresholdMode : uint8_t {
  THRESHOLD_MIN_X3 = 0,          /* threshold = min * 3.0           */
  THRESHOLD_MID_X1_04 = 1        /* threshold = ((min+max)/2) * 1.04 */
};

/* One robot variant.  Adding a robot = adding one Profile (see docs/). */
struct Profile {
  uint8_t  irCount;              /* number of IR sensors                    */
  uint16_t irMask;               /* (1 << irCount) - 1                      */
  uint8_t  irCenter;             /* index of the centre sensor              */
  uint8_t  irOnDefault;          /* level that switches the IR emitters on  */
  bool     hasTurnType3;         /* turnType 3 exists (HammerHead only)     */
  bool     hasLineAggregation;   /* 11 sensors folded into a 5-bit pattern  */
  uint8_t  turnT;                /* turnType set by a T junction            */
  uint8_t  turnSide;             /* turnType set by a LEFT/RIGHT junction   */
  uint8_t  spdUpMs;              /* speed ramp: ms between steps            */
  uint8_t  spdUpStep;            /* speed ramp: step size                   */
  uint8_t  spdLo;                /* lowest line-following speed             */
  uint8_t  spdHi;                /* highest *starting* speed                */
  uint8_t  spdTargetMin;         /* lowest accepted target speed            */
  uint8_t  actSpdT1;             /* default turn speed, turnType 1          */
  uint8_t  actSpdT2;             /* default turn speed, turnType 2          */
  uint8_t  actSpdT3;             /* default turn speed, turnType 3          */
  uint8_t  traceLo;              /* trace-back speed at the low end         */
  uint8_t  traceHi;              /* trace-back speed at the high end        */
  uint8_t  actSpdLo;             /* lowest accepted turn speed              */
  uint8_t  turnLeftLoBit;        /* sensor bits that end a left turn        */
  uint8_t  turnLeftHiBit;
  uint8_t  turnRightLoBit;       /* sensor bits that end a right turn       */
  uint8_t  turnRightHiBit;
  bool     yThreePatterns;       /* Y junction uses 3 patterns (H5: 2)      */
  bool     tTwoPatterns;         /* T junction uses 2 patterns (H5: 1)      */
  bool     limitTraceSpeed;      /* clamp speed to 150 after a trace-back   */
  ThresholdMode thresholdMode;
};

extern const Profile kHammerHead5;
extern const Profile kHammerHead11;
extern const Profile kComel2;
extern const Profile kComel3;

/* Arduino's map(): integer arithmetic, no clamping (it extrapolates). */
long arduinoMap(long x, long inMin, long inMax, long outMin, long outMax);

/* ---- Sensors ------------------------------------------------------------ */

/* Pack sensor readings into a word: bit i is set when sensor i "sees" the
 * line.  lineColor == false: black line (reading > threshold);
 * lineColor == true: white line (reading < threshold). */
uint16_t irWord(const Profile &p, const uint8_t *values,
                const uint8_t *thresholds, bool lineColor);

/* HammerHead_11: fold the 11-bit word into the 5-bit pattern used by the
 * junction detectors (left_2, left_1, mid, right_1, right_2). */
uint8_t aggregatePattern(uint16_t ir);

/* The pattern that the junction detectors look at for this profile. */
uint8_t junctionPattern(const Profile &p, uint16_t ir);

/* Weighted line position: 0 when centred, negative when the line is to the
 * left (high sensor indices), positive when it is to the right. */
int16_t computeError(const Profile &p, uint16_t ir);

/* 0b10 = last seen on the left, 0b01 = last seen on the right.  The previous
 * value is kept when both or neither outer sensor sees the line. */
uint8_t updateLastIR(const Profile &p, uint16_t ir, uint8_t lastIR);

/* ---- Junction patterns -------------------------------------------------- */
bool isYJunction(const Profile &p, uint8_t pattern);
bool isTJunction(const Profile &p, uint8_t pattern);
bool isLeftJunction(uint8_t pattern);
bool isRightJunction(uint8_t pattern);

/* ---- PID (PD) ----------------------------------------------------------- */
struct PidState {
  int32_t  prevError;
  uint32_t prevTimeMs;
};
struct MotorPair {
  int16_t left;                  /* signed speed for motor A */
  int16_t right;                 /* signed speed for motor B */
};

/* One PD step.  control = KP*error + KD*(d error / d t[ms]).  The motor
 * speeds are limited to [-spd, +spd].  dt <= 0 gives a zero derivative. */
MotorPair pidStep(PidState &s, uint8_t spd, int16_t error, float kp, float kd,
                  uint32_t nowMs);

/* ---- Junction-following state machine ----------------------------------- */
enum class StepKind : uint8_t {
  Lost,                          /* no sensor on the line                   */
  Follow,                        /* on the line: run the PD controller      */
  Detected                       /* the requested junction was found        */
};

struct JunctionState {
  uint16_t spd;                  /* current commanded speed                 */
  uint8_t  spdTarget;            /* speed to ramp up to                     */
  uint32_t timerSpdUp;           /* last ramp step (persists between calls) */
  uint32_t enterTime;            /* when junction() was entered             */
  uint8_t  lastIR;               /* 0b10 / 0b01                             */
  uint8_t  turnType;             /* result: 1, 2 or 3                       */
};

struct StepResult {
  StepKind kind;
  int16_t  error;                /* Follow                                   */
  uint16_t spd;                  /* Follow, or Lost with gap == true         */
  uint8_t  traceSpd;             /* Lost with gap == false                   */
  bool     traceLeft;            /* Lost with gap == false                   */
  bool     driveForward;         /* Lost with gap == true                    */
};

void junctionBegin(const Profile &p, JunctionState &st, uint16_t spd,
                   uint32_t nowMs, uint8_t lastIR, uint32_t timerSpdUp);

/* type: 'Y', 'T', 'L', 'R' or 'S' (first character of the type string). */
StepResult junctionStep(const Profile &p, JunctionState &st, char type,
                        int16_t skipDuration, bool gap, uint16_t ir,
                        uint32_t nowMs);

/* ---- Turning at a junction ---------------------------------------------- */
struct TurnPlan {
  bool     valid;                /* false: not a LEFT/RIGHT request -> stop  */
  bool     move;                 /* false: unknown turnType, keep motors     */
  bool     right;
  uint16_t spdL;                 /* arguments for KILAT_movement()           */
  uint16_t spdR;
  uint16_t settleMs;             /* delay before looking for the line        */
};
/* dir: first character of the action string ('L' or 'R'). */
TurnPlan planTurn(const Profile &p, uint8_t turnType, char dir,
                  uint8_t requestedSpd);

/* True when the sensors have reacquired the line after a turn. */
bool turnReacquired(const Profile &p, bool turnLeft, uint16_t ir);

/* ---- Calibration, buttons, battery, display ----------------------------- */
uint8_t thresholdFromRange(const Profile &p, uint8_t minV, uint8_t maxV);

/* bit 2 = START, bit 1 = UP, bit 0 = DOWN */
uint8_t decodeButtons(uint8_t upDownRaw, uint8_t startRaw);

uint8_t batteryPercent(uint8_t raw);   /* 0..100 */
uint8_t batteryLevel(uint8_t percent); /* 0..14  */

/* Writes "S.CCsec" (seconds, two-digit hundredths).  Returns the length. */
uint8_t formatElapsed(uint32_t ms, char *out, uint8_t capacity);

}  /* namespace kilat */

#endif /* KILAT_CORE_H */
