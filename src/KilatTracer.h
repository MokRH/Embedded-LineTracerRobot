/*
 * KilatTracer.h
 * =============================================================================
 * Public API of the KILAT line-following robot library
 * (HammerHead_5, HammerHead_11, Comel2, Comel3 on ATmega328P).
 *
 *   KilatTracer.h  - this file: configuration, pins, globals, prototypes
 *   KilatTracer.cpp- hardware layer (pins, ADC, PWM, OLED, EEPROM, delays)
 *   KilatCore.*    - hardware-independent logic, unit-tested on a PC
 *   KilatConfig.h  - the robot type and optional features
 *
 * See README.md and docs/ARCHITECTURE.md.
 * =============================================================================
 */
#ifndef KILATTRACER_H
#define KILATTRACER_H

#define KILAT_VERSION_MAJOR 2
#define KILAT_VERSION_MINOR 0
#define KILAT_VERSION_PATCH 0

#include "KilatConfig.h"

/* ========================================================================== *
 *  1.  CONFIGURATION CHECKS
 * ========================================================================== */

/* Nothing selected: default to HammerHead_5 (original behaviour). */
#if !defined(HammerHead_5) && !defined(HammerHead_11) && \
    !defined(Comel2) && !defined(Comel3)
  #define HammerHead_5
#endif

#if (defined(HammerHead_5)  + defined(HammerHead_11) + \
     defined(Comel2)        + defined(Comel3)) > 1
  #error "KILAT: only ONE robot type may be defined at a time (HammerHead_5, HammerHead_11, Comel2 or Comel3)."
#endif

/* The grippers are on by default; switch them off with Servo1_OFF/Servo2_OFF. */
#if !defined(Servo1_OFF) && !defined(Servo1_ON)
  #define Servo1_ON
#endif
#if !defined(Servo2_OFF) && !defined(Servo2_ON)
  #define Servo2_ON
#endif

/* ========================================================================== *
 *  2.  ROBOT PROFILE
 * ========================================================================== *
 *  The tunable numbers of each robot live in KilatCore.cpp (struct Profile).
 *  Only the few values needed for array sizes are repeated here.
 * ========================================================================== */
#include "KilatCore.h"

#if defined(HammerHead_11)
  #define KILAT_PROFILE        (kilat::kHammerHead11)
  #define KILAT_IR_COUNT       11u
  #define KILAT_IR_MASK        0x07FFu
  #define KILAT_IR_CENTER      5u
  #define KILAT_IR_ON_DEFAULT  1u
#elif defined(HammerHead_5)
  #define KILAT_PROFILE        (kilat::kHammerHead5)
  #define KILAT_IR_COUNT       5u
  #define KILAT_IR_MASK        0x001Fu
  #define KILAT_IR_CENTER      2u
  #define KILAT_IR_ON_DEFAULT  0u
#elif defined(Comel2)
  #define KILAT_PROFILE        (kilat::kComel2)
  #define KILAT_IR_COUNT       5u
  #define KILAT_IR_MASK        0x001Fu
  #define KILAT_IR_CENTER      2u
  #define KILAT_IR_ON_DEFAULT  0u
#elif defined(Comel3)
  #define KILAT_PROFILE        (kilat::kComel3)
  #define KILAT_IR_COUNT       5u
  #define KILAT_IR_MASK        0x001Fu
  #define KILAT_IR_CENTER      2u
  #define KILAT_IR_ON_DEFAULT  0u
#endif

#define KILAT_EEPROM_BYTES  KILAT_IR_COUNT

/* ========================================================================== *
 *  3.  INCLUDES
 * ========================================================================== */
#include <Arduino.h>
#include <EEPROM.h>

/* Optional libraries.  The extern objects are visible to sketches (e.g. `u8x8`,
 * `btSerial`, `huskylens`). */
#ifndef oled_OFF
  #include <U8x8lib.h>                       /* library "U8g2" */
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
  #include "HUSKYLENS.h"                     /* library "HUSKYLENS" (DFRobot) */
  extern HUSKYLENS huskylens;
#endif

/* ========================================================================== *
 *  4.  PIN DEFINITIONS  (extern)
 * ========================================================================== */
extern const uint8_t KILAT_PIN_IR_ON;
extern const uint8_t KILAT_PIN_BUZZ;

/* [green, yellow, red].  NOTE: pins 0 and 1 are also the hardware serial port,
 * so Serial cannot be used for debugging on these robots. */
extern const uint8_t KILAT_Digi_write[3];

/* [pwm_A+, pwm_A-, pwm_B+, pwm_B-]   (A = left motor, B = right motor) */
extern const uint8_t KILAT_Motor[4];

/* Multiplexer select [s0, s1, s2] and analog read pins.
 * Mux_read always has 2 entries (index 1 unused on Comel). */
extern const uint8_t KILAT_Mux_sel[3];
extern const uint8_t KILAT_Mux_read[2];

/* Logical IR channel indices, physical order from one side to the other. */
extern const uint8_t KILAT_IRnumber[KILAT_IR_COUNT];

/* ========================================================================== *
 *  5.  MUX CHANNEL CONSTANTS
 * ========================================================================== */
#define MUX_CH_UP_DOWN    20
#define MUX_CH_START_BTN  21
#define MUX_CH_BATT_MON   22
#define KILAT_MUX_TABLE_SIZE 32u

/* ========================================================================== *
 *  6.  PUBLIC GLOBALS
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
 *  7.  FUNCTION PROTOTYPES
 * ========================================================================== */

/* --- Mux --------------------------------------------------------------- */
uint8_t readMux(uint8_t item);
uint8_t KILAT_muxSelectByte(uint8_t item);   /* diagnostics/tests: 0xFF = unused */

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

/* --- Optional: grippers (PWM on pins 9 / 10, 1-2 ms pulses) ----------- */
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
