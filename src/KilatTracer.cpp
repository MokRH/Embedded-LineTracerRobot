/*
 * KilatTracer.cpp
 * =============================================================================
 * Hardware layer of the KILAT line-following robot library: pins, multiplexer,
 * ADC, PWM, OLED, EEPROM and the blocking loops.  All decisions (line position,
 * junction detection, PD control, turn planning, thresholds, ...) are made by
 * the pure functions in KilatCore.cpp, which are unit-tested on a PC.
 *
 * See KilatTracer.h for the public API and KilatConfig.h for the build options.
 * =============================================================================
 */
#include "KilatTracer.h"

/* ========================================================================== *
 *  1.  PIN DEFINITIONS
 * ========================================================================== */
const uint8_t KILAT_PIN_IR_ON = A3;
const uint8_t KILAT_PIN_BUZZ  = 4;

const uint8_t KILAT_Digi_write[3] = { 0, 1, 2 };          /* green, yellow, red */
const uint8_t KILAT_Motor[4]      = { 3, 11, 5, 6 };      /* pwmA+, pwmA-, pwmB+, pwmB- */

#if defined(HammerHead_5) || defined(HammerHead_11)
  const uint8_t KILAT_Mux_sel[3]  = { A1, 13, A0 };       /* s0, s1, s2 */
  const uint8_t KILAT_Mux_read[2] = { A6, A7 };
#elif defined(Comel2) || defined(Comel3)
  const uint8_t KILAT_Mux_sel[3]  = { A2, A0, A1 };
  const uint8_t KILAT_Mux_read[2] = { A7, A7 };           /* only index 0 is used */
#endif

#if defined(HammerHead_11)
  const uint8_t KILAT_IRnumber[KILAT_IR_COUNT] = { 10, 9, 8, 7, 6,
                                                     5,
                                                    4, 3, 2, 1, 0 };
#elif defined(HammerHead_5)
  const uint8_t KILAT_IRnumber[KILAT_IR_COUNT] = { 7, 6, 5, 4, 3 };
#elif defined(Comel2) || defined(Comel3)
  const uint8_t KILAT_IRnumber[KILAT_IR_COUNT] = { 4, 3, 2, 1, 0 };
#endif


/* ========================================================================== *
 *  2.  MUX  (PROGMEM lookup table)
 * ========================================================================== *
 *  Index = logical channel. Value = 6-bit select byte.
 *    bits 0..2 : Mux_sel[0..2]
 *    bits 4..5 : which Mux_read pin (0x10 -> read[1], 0x20 -> read[0])
 *  0xFF marks an unused slot.
 * ========================================================================== */
#if defined(HammerHead_5) || defined(HammerHead_11)
static const uint8_t kMuxTable[KILAT_MUX_TABLE_SIZE] PROGMEM = {
  /* 0  */ 0x10, 0x13, 0x14, 0x16, 0x17, 0x15, 0x25, 0x27,   /* IR 0..7   */
  /* 8  */ 0x26, 0x24, 0x11, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,   /* IR 8..10  */
  /* 16 */ 0xFF, 0xFF, 0xFF, 0xFF, 0x20, 0x21, 0x22, 0xFF,   /* 20,21,22  */
  /* 24 */ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};
#elif defined(Comel2) || defined(Comel3)
static const uint8_t kMuxTable[KILAT_MUX_TABLE_SIZE] PROGMEM = {
  /* 0  */ 0x04, 0x06, 0x07, 0x05, 0x02, 0xFF, 0xFF, 0xFF,   /* IR 0..4   */
  /* 8  */ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  /* 16 */ 0xFF, 0xFF, 0xFF, 0xFF, 0x03, 0x00, 0x01, 0xFF,   /* 20,21,22  */
  /* 24 */ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};
#endif

uint8_t KILAT_muxSelectByte(uint8_t item) {
  if (item >= KILAT_MUX_TABLE_SIZE) return 0xFF;
  return pgm_read_byte(&kMuxTable[item]);
}

uint8_t readMux(uint8_t item) {
  const uint8_t select_val = KILAT_muxSelectByte(item);
  if (select_val == 0xFF) return 0;

  digitalWrite(KILAT_Mux_sel[0], bitRead(select_val, 0));
  digitalWrite(KILAT_Mux_sel[1], bitRead(select_val, 1));
  digitalWrite(KILAT_Mux_sel[2], bitRead(select_val, 2));

  const uint8_t readIdx = (select_val & 0x20) ? 0u : 1u;
  return (uint8_t)map(analogRead(KILAT_Mux_read[readIdx]), 0, 1023, 0, 255);
}


/* ========================================================================== *
 *  3.  MOTOR
 * ========================================================================== */
uint8_t turnType = 1;   /* default: turn at centre */

void motor_A(int16_t spdA) {
  spdA = constrain(spdA, -255, 255);
  if (spdA >= 0) {
    analogWrite(KILAT_Motor[0], (uint8_t)spdA);
    digitalWrite(KILAT_Motor[1], LOW);
  } else {
    analogWrite(KILAT_Motor[1], (uint8_t)(-spdA));
    digitalWrite(KILAT_Motor[0], LOW);
  }
}

void motor_B(int16_t spdB) {
  spdB = constrain(spdB, -255, 255);
  if (spdB >= 0) {
    analogWrite(KILAT_Motor[2], (uint8_t)spdB);
    digitalWrite(KILAT_Motor[3], LOW);
  } else {
    analogWrite(KILAT_Motor[3], (uint8_t)(-spdB));
    digitalWrite(KILAT_Motor[2], LOW);
  }
}

void KILAT_movement(const char *dir, uint16_t spdL, uint16_t spdR) {
  spdL = constrain(spdL, 0, 255);
  spdR = constrain(spdR, 0, 255);

  switch (dir[0]) {
    case 'F':   /* FWD */
      motor_A(spdL);  motor_B(spdR);
      break;
    case 'R':
      if (dir[1] == 'E') {            /* REV */
        motor_A(-(int16_t)spdL);  motor_B(-(int16_t)spdR);
      } else {                        /* RIGHT */
        motor_A(spdL);  motor_B(-(int16_t)spdR);
      }
      break;
    case 'L':   /* LEFT */
      motor_A(-(int16_t)spdL);  motor_B(spdR);
      break;
    case 'S':   /* STOP */
    default:
      motor_A(0);  motor_B(0);
      break;
  }
  turnType = 1;
}


/* ========================================================================== *
 *  4.  LED
 * ========================================================================== */
void KILAT_LED(const char *led, bool ON_OFF) {
  uint8_t idx;
  switch (led[0]) {
    case 'r': idx = 2; break;   /* red    */
    case 'y': idx = 1; break;   /* yellow */
    case 'g': idx = 0; break;   /* green  */
    default:  return;
  }
  digitalWrite(KILAT_Digi_write[idx], ON_OFF ? HIGH : LOW);
}


/* ========================================================================== *
 *  5.  BUTTON
 * ========================================================================== */
uint8_t KILAT_getButton(void) {
  const uint8_t upDown = readMux(MUX_CH_UP_DOWN);
  const uint8_t start  = readMux(MUX_CH_START_BTN);
  const uint8_t state  = kilat::decodeButtons(upDown, start);

  delay(20);   /* cheap debounce */
  return state;
}


/* ========================================================================== *
 *  6.  IR  (calibration + read-out with SRAM-cached thresholds)
 * ========================================================================== */
uint8_t g_IR_threshold[KILAT_EEPROM_BYTES];

void KILAT_IRinit(void) {
  for (uint8_t i = 0; i < KILAT_EEPROM_BYTES; i++)
    g_IR_threshold[i] = EEPROM.read(i);
}

void KILAT_IRcalibrate(void) {
  /* Only run when UP is held and the battery is healthy. */
  if (KILAT_getButton() != 0b00000010) return;
  if (readMux(MUX_CH_BATT_MON) <= 100) return;

  uint8_t IR_min[KILAT_IR_COUNT];
  uint8_t IR_max[KILAT_IR_COUNT];
  for (uint8_t i = 0; i < KILAT_IR_COUNT; i++) {
    IR_min[i] = 255;
    IR_max[i] = 0;
  }

  /* Blink all 3 LEDs to tell the user calibration is about to start. */
  while (KILAT_getButton() == 0b00000010) {
    KILAT_LED("green", 1); KILAT_LED("yellow", 0); KILAT_LED("red", 0);
    delay(200);
    KILAT_LED("green", 0); KILAT_LED("yellow", 1); KILAT_LED("red", 0);
    delay(200);
    KILAT_LED("green", 0); KILAT_LED("yellow", 0); KILAT_LED("red", 1);
    delay(200);
  }
  KILAT_LED("green", 0); KILAT_LED("yellow", 0); KILAT_LED("red", 0);

  oledDisplay_char(1, 0, "READING");
  oledDisplay_char(2, 1, "OFFSET");
  delay(1000);

  /* Sweep forward for half the window, reverse for the other half. */
  uint8_t dispCount = 13, dispShift = 0;
  for (uint8_t j = 0; j < 200; j++) {
    if (j < 100) KILAT_movement("FWD", 80, 80);
    else         KILAT_movement("REV", 80, 80);

    for (uint8_t i = 0; i < KILAT_IR_COUNT; i++) {
      const uint8_t v = readMux(KILAT_IRnumber[i]);
      if (v < IR_min[i]) IR_min[i] = v;
      if (v > IR_max[i]) IR_max[i] = v;
    }

    /* Progress bar: draw one '-' every 14 samples. */
    if (++dispCount == 14) {
      oledDisplay_char(dispShift, 3, "-");
      dispShift++;
      dispCount = 0;
    }
    delay(5);
  }

  KILAT_movement("STOP", 0, 0);

  /* Compute and persist thresholds (the rule differs per robot; see
   * kilat::thresholdFromRange). */
  for (uint8_t i = 0; i < KILAT_IR_COUNT; i++) {
    const uint8_t t = kilat::thresholdFromRange(KILAT_PROFILE, IR_min[i], IR_max[i]);
    EEPROM.write(i, t);
    g_IR_threshold[i] = t;   /* keep cache in sync */
  }

#ifndef oled_OFF
  oledDisplay_char(0, 0, "        "); oledDisplay_char(2, 0, "OFFSET");
  oledDisplay_char(0, 1, "        "); oledDisplay_char(3, 1, "SAVED");

  /* "val:NNN " - built by hand, no heap-allocating String. */
  char getChar[9] = { 'v', 'a', 'l', ':', 0, 0, 0, 0, 0 };
  {
    uint8_t n = 4;
    const uint8_t v = g_IR_threshold[0];
    if (v >= 100) getChar[n++] = (char)('0' + v / 100);
    if (v >= 10)  getChar[n++] = (char)('0' + (v / 10) % 10);
    getChar[n++] = (char)('0' + v % 10);
    getChar[n++] = ' ';
    getChar[n] = '\0';
  }
  oledDisplay_char(0, 3, getChar);
  delay(1000);
  oledDisplay_clear();
#endif
}

uint16_t KILAT_IRupdate(bool line_color) {
  uint8_t values[KILAT_IR_COUNT];
  for (uint8_t i = 0; i < KILAT_IR_COUNT; i++)
    values[i] = readMux(KILAT_IRnumber[i]);
  return kilat::irWord(KILAT_PROFILE, values, g_IR_threshold, line_color);
}


/* ========================================================================== *
 *  7.  PID
 * ========================================================================== *
 *  control = KP * error + KD * (d error / d t[ms])   (float dt, guarded)
 *  The maths lives in kilat::pidStep().
 * ========================================================================== */
static kilat::PidState g_pid = { 0, 0 };

void KILAT_PID(uint8_t spd, int16_t current_error, float KP, float KD) {
  const kilat::MotorPair m = kilat::pidStep(g_pid, spd, current_error, KP, KD, millis());
  motor_A(m.left);
  motor_B(m.right);
}


/* ========================================================================== *
 *  8.  OLED
 * ========================================================================== */
#ifndef oled_OFF
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);

/* ---- Bitmap glyphs (PROGMEM) -------------------------------------------- */
static const uint8_t kBattOutline[16] PROGMEM = {
  0xfc, 0x04, 0x07, 0x01, 0x01, 0x07, 0x04, 0xfc,
  0xff, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0xff
};
static const uint8_t kLowBatt[16]    PROGMEM = {
  0x00, 0x9f, 0x50, 0x50, 0x50, 0x80, 0x00, 0x00,
  0x00, 0x73, 0x84, 0x64, 0x84, 0x73, 0x00, 0x00
};
static const uint8_t kBarBase[24]    PROGMEM = {
  0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00
};
#endif

void oledDisplay_setup() {
#ifndef oled_OFF
  u8x8.begin();
  u8x8.setPowerSave(0);
  u8x8.setFont(u8x8_font_lucasarts_scumm_subtitle_r_2x2_f);
#if defined(HammerHead_5) || defined(HammerHead_11)
  u8x8.setFlipMode(1);
#endif
#if defined(Comel2) || defined(Comel3)
  u8x8.setFlipMode(0);
#endif
#endif
}

void oledDisplay_char(uint8_t column, uint8_t line, const char *string) {
#ifndef oled_OFF
  u8x8.drawString(column, line * 2, (char *)string);
#else
  (void)column; (void)line; (void)string;
#endif
}

void oledDisplay_clear(void) {
#ifndef oled_OFF
  u8x8.clear();
#endif
}

void oledDisplay_showName() {
#ifndef oled_OFF
#if defined(HammerHead_5) || defined(HammerHead_11)
  oledDisplay_char(0, 0, " Hammer ");
  oledDisplay_char(0, 1, "  Head  ");
#elif defined(Comel2)
  oledDisplay_char(3, 0, "Kilat");
  oledDisplay_char(2, 1, "Comel2");
#elif defined(Comel3)
  oledDisplay_char(3, 0, "Kilat");
  oledDisplay_char(2, 1, "Comel3");
#endif
#endif
}

void oledDisplay_showBatt() {
#ifndef oled_OFF
  const uint8_t battPercent = kilat::batteryPercent(readMux(MUX_CH_BATT_MON));
  const uint8_t level = kilat::batteryLevel(battPercent);

  static uint8_t level_old = 15;
  if (level == level_old) return;
  level_old = level;

  uint8_t drawBatt[16];
  memcpy_P(drawBatt, kBattOutline, sizeof(drawBatt));

  if (level > 2) {
    uint8_t fill = level - 1;
    for (uint8_t j = 0; j < 8; j++) {
      for (uint8_t i = 0; i < fill && i < 14; i++) {
        if (i < 7)      bitSet(drawBatt[8 + j], 7 - i);
        else            bitSet(drawBatt[0 + j], 14 - i);
      }
    }
  } else {
    memcpy_P(drawBatt, kLowBatt, sizeof(drawBatt));
  }

  u8x8.drawTile(0, 6, 1, drawBatt);
  u8x8.drawTile(0, 7, 1, drawBatt + 8);
#endif
}

void oledDisplay_showBar(uint8_t pos_y, uint8_t pos_x, uint8_t percent) {
#ifndef oled_OFF
  uint8_t drawBar[24];
  memcpy_P(drawBar, kBarBase, sizeof(drawBar));

  percent = constrain(percent, 0, 100);
  const uint8_t level = map(percent, 0, 100, 0, 24);

  for (uint8_t j = 0; j < 7; j++) {
    for (uint8_t i = 0; i < level; i++) {
      if      (i <  7) bitSet(drawBar[16 + j], 7 - i);
      else if (i < 14) bitSet(drawBar[ 8 + j], 14 - i);
      else             bitSet(drawBar[ 0 + j], 21 - i);
    }
  }

  u8x8.drawTile(pos_y, pos_x - 2, 1, drawBar);
  u8x8.drawTile(pos_y, pos_x - 1, 1, drawBar + 8);
  u8x8.drawTile(pos_y, pos_x,     1, drawBar + 16);
#else
  (void)pos_y; (void)pos_x; (void)percent;
#endif
}

void oledDisplay_showIR() {
#ifndef oled_OFF
  const uint8_t Ub = 255;
  for (uint8_t i = 0; i < KILAT_IR_COUNT; i++) {
    const uint8_t getValue = constrain(readMux(KILAT_IRnumber[i]), g_IR_threshold[i], Ub);
    oledDisplay_showBar((KILAT_IR_CENTER + 7) - i, 7,
                        map(getValue, g_IR_threshold[i], Ub, 0, 100));

    const bool onLine = (getValue > g_IR_threshold[i]);
    if      (i == KILAT_IR_COUNT - 1)  KILAT_LED("green",  onLine);   /* far left  */
    else if (i == KILAT_IR_CENTER)     KILAT_LED("yellow", onLine);   /* centre    */
    else if (i == 0)                   KILAT_LED("red",    onLine);   /* far right */
  }
#endif
}

static uint32_t oled_startTime = 0;

void oledDisplay_time(uint8_t StartStop) {
  if (StartStop == 0) {
    oledDisplay_char(0, 0, "        ");
    oledDisplay_char(3, 0, "Start");
    oledDisplay_char(0, 1, "  Time  ");
    oledDisplay_char(0, 3, "0.00 sec");
    oled_startTime = millis();
    return;
  }

  /* Finish: format the time once ("S.CCsec"), then blink forever. */
  char getChar[12];
  kilat::formatElapsed(millis() - oled_startTime, getChar, sizeof(getChar));

  oledDisplay_char(0, 0, " Finish ");
  oledDisplay_char(0, 3, getChar);

  KILAT_movement("STOP", 0, 0);
  while (1) {
    for (uint8_t i = 0; i < 2; i++) {
      KILAT_LED("green", 0); KILAT_LED("red", 0); KILAT_LED("yellow", 0);
      delay(100);
      KILAT_LED("green", 1); KILAT_LED("red", 1); KILAT_LED("yellow", 0);
      delay(50);
    }
    KILAT_LED("green", 0); KILAT_LED("red", 0); KILAT_LED("yellow", 1);
    delay(300);
  }
}


/* ========================================================================== *
 *  9.  JUNCTION  (the decisions live in kilat::junctionStep / planTurn)
 * ========================================================================== */
uint8_t last_IR = 0b01;          /* 0b10 = last seen left, 0b01 = right */
static uint32_t timer_spdUP = 0; /* speed-ramp timer, persists between calls */

void action(const char *action_, uint8_t action_spd, uint16_t action_duration, bool line_color) {
  const kilat::TurnPlan plan = kilat::planTurn(KILAT_PROFILE, turnType, action_[0], action_spd);
  if (!plan.valid) {
    KILAT_movement("STOP", 0, 0);
    return;
  }

  if (plan.move)
    KILAT_movement(plan.right ? "RIGHT" : "LEFT", plan.spdL, plan.spdR);

  delay(plan.settleMs);

  if (action_duration == 0) {
    while (1) {
      const uint16_t get_IR = KILAT_IRupdate(line_color) & KILAT_IR_MASK;
      if (kilat::turnReacquired(KILAT_PROFILE, !plan.right, get_IR)) {
        last_IR = plan.right ? 0b01 : 0b10;
        break;
      }
    }
  } else {
    delay(action_duration);
  }

  KILAT_movement("STOP", 0, 0);
  delay(25);
}

void junction(const char *junc_type, uint16_t spd, float KP, float KD,
              int16_t skipDuration, bool gap, bool line_color) {
  kilat::JunctionState st;
  kilat::junctionBegin(KILAT_PROFILE, st, spd, millis(), last_IR, timer_spdUP);
  const char jType = junc_type[0];   /* 'Y', 'T', 'L', 'R', 'S' */

  while (1) {
    const uint16_t get_IR = KILAT_IRupdate(line_color) & KILAT_IR_MASK;
    KILAT_LED("green", 0); KILAT_LED("yellow", 0); KILAT_LED("red", 0);

    const kilat::StepResult r =
        kilat::junctionStep(KILAT_PROFILE, st, jType, skipDuration, gap, get_IR, millis());
    last_IR     = st.lastIR;
    timer_spdUP = st.timerSpdUp;

    switch (r.kind) {
      case kilat::StepKind::Lost:
        if (r.driveForward)      KILAT_movement("FWD", r.spd, r.spd);
        else if (r.traceLeft)    KILAT_movement("LEFT",  r.traceSpd, r.traceSpd);
        else                     KILAT_movement("RIGHT", r.traceSpd, r.traceSpd);
        break;
      case kilat::StepKind::Follow:
        KILAT_PID((uint8_t)r.spd, r.error, KP, KD);
        break;
      case kilat::StepKind::Detected:
        turnType = st.turnType;
        return;
    }
  }
}


/* ========================================================================== *
 *  10.  OPTIONAL: BLUETOOTH
 * ========================================================================== */
#ifdef BT_ON
SoftwareSerial btSerial(7, 8);   /* arRx, arTx */

void KILAT_btSetup() {
  btSerial.begin(115200);
}

void KILAT_btRename(const char *get_btChar) {
  btSerial.write("AT+NAME");
  btSerial.write(get_btChar);
  btSerial.write("\r\n");
  delay(100);
}

char KILAT_getBT() {
  while (btSerial.available() > 0)
    btSerial.read();
  while (1) {
    if (btSerial.available() > 0)
      return (char)btSerial.read();
  }
}
#endif


/* ========================================================================== *
 *  11.  OPTIONAL: CAMERA
 * ========================================================================== */
#ifdef camera_ON
SoftwareSerial cameraSerial(12, 10);   /* auxRx, auxTx */

void KILAT_cameraSetup() {
  cameraSerial.begin(115200);
}

char KILAT_getCamera(char defaultImage, uint16_t maxDuration) {
  turnType = 1;
  const uint32_t enterTime = millis();

  while (cameraSerial.available() > 0)
    cameraSerial.read();

  while (1) {
    if (cameraSerial.available() > 0)
      return (char)cameraSerial.read();
    if ((millis() - enterTime) > maxDuration)
      return defaultImage;
  }
}
#endif


/* ========================================================================== *
 *  12.  OPTIONAL: HUSKYLENS
 * ========================================================================== */
#ifdef Husky_ON
HUSKYLENS huskylens;

void KILAT_HuskySetup() {
  Wire.begin();
  while (!huskylens.begin(Wire)) {
    /* Spin until the HUSKYLENS answers. */
  }
}

uint8_t KILAT_getHusky() {
  if (!huskylens.request())     return 0;
  if (!huskylens.isLearned())   return 0;
  if (!huskylens.available())   return 0;

  uint8_t result = 0;
  while (huskylens.available()) {
    result = huskylens.read().ID;
  }
  return result;
}
#endif


/* ========================================================================== *
 *  13.  OPTIONAL: GRIPPERS  (PWM pulses on pins 9 / 10; the Servo library is
 *       not used)
 * ========================================================================== */
#ifdef Servo1_ON
void KILAT_gripper_1(uint8_t angle, uint16_t gripperDelay) {
  static bool pinMode_9 = false;
  if (!pinMode_9) {
    pinMode(9, OUTPUT);
    pinMode_9 = true;
  }
  analogWrite(9, map(angle, 0, 180, 125, 250));   /* pwm 125 = 1 ms, 250 = 2 ms */
  delay(gripperDelay);
  turnType = 1;
}
#endif

#ifdef Servo2_ON
void KILAT_gripper_2(uint8_t angle, uint16_t gripperDelay) {
  static bool pinMode_10 = false;
  if (!pinMode_10) {
    pinMode(10, OUTPUT);
    pinMode_10 = true;
  }
  analogWrite(10, map(angle, 0, 180, 125, 250));  /* pwm 125 = 1 ms, 250 = 2 ms */
  delay(gripperDelay);
  turnType = 1;
}
#endif


/* ========================================================================== *
 *  14.  SETUP + WAIT-TO-START
 * ========================================================================== */

/* Helper: configure an array of pins as OUTPUT (or INPUT) in one go. */
template <size_t N>
static inline void configurePins(const uint8_t (&pins)[N], uint8_t mode, bool initLow) {
  for (size_t i = 0; i < N; i++) {
    pinMode(pins[i], mode);
    if (mode == OUTPUT && initLow) digitalWrite(pins[i], LOW);
  }
}

void KILAT_waitToStart(void) {
  oledDisplay_showName();

  while (1) {
    oledDisplay_showBatt();
    oledDisplay_showIR();

    const uint8_t getButton = KILAT_getButton();
    if (getButton == 0b010) {                  /* UP    -> tiny forward nudge */
      KILAT_movement("FWD", 100, 100);
      while (KILAT_getButton() == 0b010) { }
      KILAT_movement("STOP", 0, 0);
    } else if (getButton == 0b001) {           /* DOWN  -> tiny reverse nudge */
      KILAT_movement("REV", 100, 100);
      while (KILAT_getButton() == 0b001) { }
      KILAT_movement("STOP", 0, 0);
    } else if (getButton == 0b100) {           /* START -> exit loop         */
      break;
    }
  }

  oledDisplay_char(0, 2, "        ");
  oledDisplay_char(0, 3, "        ");
  oledDisplay_char(3, 3, "READY");
  while (KILAT_getButton() != 0b000) { }
}

void KILAT_setup(void) {
  /* ---- Pin configuration ---- */
  configurePins(KILAT_Mux_read,   INPUT,  false);
  configurePins(KILAT_Mux_sel,    OUTPUT, true);
  configurePins(KILAT_Motor,      OUTPUT, true);
  configurePins(KILAT_Digi_write, OUTPUT, true);

  pinMode(KILAT_PIN_BUZZ,  OUTPUT); digitalWrite(KILAT_PIN_BUZZ,  LOW);
  pinMode(KILAT_PIN_IR_ON, OUTPUT); digitalWrite(KILAT_PIN_IR_ON, KILAT_IR_ON_DEFAULT);

  /* ---- PWM timer prescalers (980 Hz on D3/D11, 976 Hz on D5/D6) ---- */
  TCCR2B = 0b00000100;   /* x64 */
  TCCR2A = 0b00000011;   /* fast PWM */
  TCCR0B = 0b00000011;   /* x64 */
  TCCR0A = 0b00000011;   /* fast PWM */

  /* ---- OLED + cached IR thresholds ---- */
  oledDisplay_setup();
  KILAT_IRinit();

  /* ---- Optional peripherals ---- */
#ifdef BT_ON
  KILAT_btSetup();
#endif
#ifdef camera_ON
  KILAT_cameraSetup();
#endif
#ifdef Husky_ON
  KILAT_HuskySetup();
#endif

  /* ---- Calibration + wait for START ---- */
  KILAT_IRcalibrate();
  KILAT_waitToStart();

  KILAT_LED("green", 0); KILAT_LED("yellow", 0); KILAT_LED("red", 0);
  oledDisplay_time(0);
}
