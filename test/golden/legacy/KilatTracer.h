/*
 * KilatTracer.h
 * =============================================================================
 * Header for the KILAT line-following robot library.
 *
 * The matching implementation lives in KilatTracer.cpp, which is compiled as
 * a separate translation unit by the Arduino IDE.
 *
 * -----------------------------------------------------------------------------
 *  Selecting a robot type
 * -----------------------------------------------------------------------------
 *  The robot type and optional features are #defined directly in this header
 *  (see the blocks below).  Because KilatTracer.cpp is compiled separately,
 *  #defines from your .ino are NOT visible to it — so you must set them HERE.
 *
 *  Supported types:  HammerHead_5, HammerHead_11, Comel2, Comel3
 *
 *  Optional features: BT_ON, camera_ON, Husky_ON, Servo1_ON, Servo2_ON, oled_OFF
 * =============================================================================
 */
#ifndef KILATTRACER_H
#define KILATTRACER_H

/* ========================================================================== *
 *  1.  CONFIGURATION
 * ========================================================================== */

/* ----------------------- Robot type (pick ONE) --------------------------- */
/* PATCH 1 (test only): the original always defined HammerHead_5, which makes a
 * build flag for another robot hit the "only ONE robot type" #error. */
#if !defined(HammerHead_11) && !defined(Comel2) && !defined(Comel3)
#ifndef HammerHead_5
 #define HammerHead_5
#endif
#endif
#ifndef HammerHead_11
// #define HammerHead_11
#endif
#ifndef Comel2
// #define Comel2
#endif
#ifndef Comel3
// #define Comel3
#endif

/* ----------------------- Optional features ------------------------------- */
#ifndef BT_ON
// #define BT_ON
#endif
#ifndef camera_ON
// #define camera_ON
#endif
#ifndef Husky_ON
// #define Husky_ON
#endif
#ifndef Servo1_ON
  #define Servo1_ON
#endif
#ifndef Servo2_ON
 #define Servo2_ON
#endif
#ifndef oled_OFF
// #define oled_OFF
#endif

/* ----------------------- Validation -------------------------------------- */
#if !defined(HammerHead_5) && !defined(HammerHead_11) && !defined(Comel2) && !defined(Comel3)
  #error "KILAT: no robot type defined. Define HammerHead_5, HammerHead_11, Comel2 or Comel3 (see kilat.h)."
#endif
#if (defined(HammerHead_5)  && defined(HammerHead_11)) || \
    (defined(HammerHead_5)  && defined(Comel2))        || \
    (defined(HammerHead_5)  && defined(Comel3))        || \
    (defined(HammerHead_11) && defined(Comel2))        || \
    (defined(HammerHead_11) && defined(Comel3))        || \
    (defined(Comel2)        && defined(Comel3))
  #error "KILAT: only ONE robot type may be defined at a time."
#endif

/* ----------------- Derived constants (per robot) ------------------------- *
 * Adding a new robot only requires adding a block here.
 * ========================================================================= */
#if defined(HammerHead_11)
  #define KILAT_IR_COUNT       11u
  #define KILAT_IR_MASK        0x07FFu
  #define KILAT_IR_CENTER      5u
  #define KILAT_IR_ON_DEFAULT  1u
  #define KILAT_HAS_TURNTYPE3  1
  #define KILAT_HAS_LINE_AGG   1
  #define KILAT_TURN_T         3
  #define KILAT_TURN_SIDE      3
  #define KILAT_SPD_UP_MS      1u
  #define KILAT_SPD_UP_STEP    2u
  #define KILAT_SPD_LO         50u
  #define KILAT_SPD_HI         150u
  #define KILAT_SPD_TARGET_MIN 50u
  #define KILAT_ACT_SPD_T1     75u
  #define KILAT_ACT_SPD_T2     100u
  #define KILAT_ACT_SPD_T3     100u
  #define KILAT_TRACE_LO       100u
  #define KILAT_TRACE_HI       200u

#elif defined(HammerHead_5)
  #define KILAT_IR_COUNT       5u
  #define KILAT_IR_MASK        0x001Fu
  #define KILAT_IR_CENTER      2u
  #define KILAT_IR_ON_DEFAULT  0u
  #define KILAT_HAS_TURNTYPE3  1
  #define KILAT_HAS_LINE_AGG   0
  #define KILAT_TURN_T         3
  #define KILAT_TURN_SIDE      3
  #define KILAT_SPD_UP_MS      2u
  #define KILAT_SPD_UP_STEP    1u
  #define KILAT_SPD_LO         25u
  #define KILAT_SPD_HI         100u
  #define KILAT_SPD_TARGET_MIN 25u
  #define KILAT_ACT_SPD_T1     75u
  #define KILAT_ACT_SPD_T2     100u
  #define KILAT_ACT_SPD_T3     100u
  #define KILAT_TRACE_LO       100u
  #define KILAT_TRACE_HI       200u

#elif defined(Comel2) || defined(Comel3)
  #define KILAT_IR_COUNT       5u
  #define KILAT_IR_MASK        0x001Fu
  #define KILAT_IR_CENTER      2u
  #define KILAT_IR_ON_DEFAULT  0u
  #define KILAT_HAS_TURNTYPE3  0
  #define KILAT_HAS_LINE_AGG   0
  #define KILAT_TURN_T         1
  #define KILAT_TURN_SIDE      2
  #define KILAT_SPD_UP_MS      2u
  #define KILAT_SPD_UP_STEP    1u
  #define KILAT_SPD_LO         25u
  #define KILAT_SPD_HI         100u
  #define KILAT_SPD_TARGET_MIN 25u
  #define KILAT_ACT_SPD_T1     60u
  #define KILAT_ACT_SPD_T2     75u
  #define KILAT_ACT_SPD_T3     0u
  #define KILAT_TRACE_LO       50u
  #define KILAT_TRACE_HI       100u
#endif

#define KILAT_EEPROM_BYTES  KILAT_IR_COUNT


/* ========================================================================== *
 *  2.  INCLUDES
 * ========================================================================== */
#include <Arduino.h>
#include <EEPROM.h>

/* Optional library includes + extern global object declarations.
 * These must be visible in the header so that user sketches can reference
 * e.g. `u8x8`, `btSerial`, `huskylens` directly (as they did in the original
 * firmware). */

#ifndef oled_OFF
  /* PATCH 2 (test only): include path of the U8g2 header */
  #include <U8x8lib.h>
  extern U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8;
#endif

#if defined(BT_ON) || defined(camera_ON)
  #include <SoftwareSerial.h>
#endif
#ifdef BT_ON
  extern SoftwareSerial btSerial;
#endif
#ifdef camera_ON
  extern SoftwareSerial cameraSerial;
#endif

#ifdef Husky_ON
  #include <Wire.h>
  #include "HUSKYLENS.h"
  extern HUSKYLENS huskylens;
#endif

#if defined(Servo1_ON) || defined(Servo2_ON)
  // #include <Servo.h>
#endif
#ifdef Servo1_ON
  // extern Servo servo1;
#endif
#ifdef Servo2_ON
  // extern Servo servo2;
#endif


/* ========================================================================== *
 *  3.  PIN DEFINITIONS  (extern)
 * ========================================================================== */
extern const uint8_t KILAT_PIN_IR_ON;
extern const uint8_t KILAT_PIN_BUZZ;

/* [green, yellow, red] */
extern const uint8_t KILAT_Digi_write[3];

/* [pwm_A+, pwm_A-, pwm_B+, pwm_B-] */
extern const uint8_t KILAT_Motor[4];

/* Multiplexer select [s0, s1, s2] and analog read pins.
 * Mux_read always has 2 entries (index 1 unused on Comel). */
extern const uint8_t KILAT_Mux_sel[3];
extern const uint8_t KILAT_Mux_read[2];

/* Logical IR channel indices, physical order from one side to the other. */
extern const uint8_t KILAT_IRnumber[KILAT_IR_COUNT];


/* ========================================================================== *
 *  4.  MUX CHANNEL CONSTANTS
 * ========================================================================== */
#define MUX_CH_UP_DOWN    20
#define MUX_CH_START_BTN  21
#define MUX_CH_BATT_MON   22
#define KILAT_MUX_TABLE_SIZE 32u


/* ========================================================================== *
 *  5.  PUBLIC GLOBALS
 * ========================================================================== */

/* Shared between movement/junction code.
 * 1 = turn at centre, 2 = bias to one side (Y-junction),
 * 3 = strong bias (HammerHead only). */
extern uint8_t turnType;

/* 0b10 = line last seen on the left, 0b01 = line last seen on the right.
 * Updated by junction()/action(); read when the robot is lost. */
extern uint8_t last_IR;

/* SRAM-cached IR thresholds, populated from EEPROM by KILAT_IRinit(). */
extern uint8_t g_IR_threshold[KILAT_EEPROM_BYTES];


/* ========================================================================== *
 *  6.  FUNCTION PROTOTYPES
 * ========================================================================== */

/* --- Mux --------------------------------------------------------------- */
uint8_t readMux(uint8_t item);

/* --- Motor ------------------------------------------------------------- */
void motor_A(int16_t spdA);
void motor_B(int16_t spdB);
void KILAT_movement(const char *dir, uint16_t spdL, uint16_t spdR);

/* --- LED --------------------------------------------------------------- */
void KILAT_LED(const char *led, bool ON_OFF);

/* --- Button ------------------------------------------------------------ */
uint8_t KILAT_getButton(void);

/* --- IR ---------------------------------------------------------------- */
void     KILAT_IRinit(void);
void     KILAT_IRcalibrate(void);
uint16_t KILAT_IRupdate(bool line_color);

/* --- PID --------------------------------------------------------------- */
void KILAT_PID(uint8_t spd, int16_t current_error, float KP, float KD);

/* --- OLED -------------------------------------------------------------- */
void oledDisplay_setup(void);
void oledDisplay_char(uint8_t column, uint8_t line, const char *string);
void oledDisplay_clear(void);
void oledDisplay_showName(void);
void oledDisplay_showBatt(void);
void oledDisplay_showBar(uint8_t pos_y, uint8_t pos_x, uint8_t percent);
void oledDisplay_showIR(void);
void oledDisplay_time(uint8_t StartStop);

/* --- Junction ---------------------------------------------------------- */
void action(const char *action_, uint8_t action_spd, uint16_t action_duration, bool line_color);
void junction(const char *junc_type, uint16_t spd, float KP, float KD,
              int16_t skipDuration, bool gap, bool line_color);

/* --- Optional: Bluetooth ---------------------------------------------- */
#ifdef BT_ON
void KILAT_btSetup(void);
void KILAT_btRename(const char *get_btChar);
char KILAT_getBT(void);
#endif

/* --- Optional: Camera -------------------------------------------------- */
#ifdef camera_ON
void KILAT_cameraSetup(void);
char KILAT_getCamera(char defaultImage, uint16_t maxDuration);
#endif

/* --- Optional: HUSKYLENS ---------------------------------------------- */
#ifdef Husky_ON
void     KILAT_HuskySetup(void);
uint8_t  KILAT_getHusky(void);
#endif

/* --- Optional: Servos -------------------------------------------------- */
#ifdef Servo1_ON
void KILAT_gripper_1(uint8_t angle, uint16_t gripperDelay);
#endif
#ifdef Servo2_ON
void KILAT_gripper_2(uint8_t angle, uint16_t gripperDelay);
#endif

/* --- Setup ------------------------------------------------------------- */
void KILAT_setup(void);
void KILAT_waitToStart(void);


#endif /* KILATTRACER_H */
