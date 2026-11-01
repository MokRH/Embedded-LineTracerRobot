/*
 * KilatConfig.h
 * =============================================================================
 * Build-time configuration of the KILAT library.
 *
 * Choose ONE robot type and any optional features.  You can do this either
 *
 *   (a) by uncommenting the lines below  (classic Arduino IDE), or
 *   (b) by passing a compiler flag        (arduino-cli / PlatformIO), e.g.
 *         arduino-cli compile --build-property "build.extra_flags=-DHammerHead_11" ...
 *         build_flags = -DHammerHead_11
 *
 * #defines in your .ino are NOT visible to the library (it is compiled as a
 * separate translation unit), so they have no effect.
 *
 * If nothing is selected the library defaults to HammerHead_5, exactly as the
 * original firmware did.
 * =============================================================================
 */
#ifndef KILAT_CONFIG_H
#define KILAT_CONFIG_H

/* ----------------------- Robot type (pick ONE) --------------------------- */
// #define HammerHead_5
// #define HammerHead_11
// #define Comel2
// #define Comel3

/* ----------------------- Optional features ------------------------------- */
// #define BT_ON        /* Bluetooth module on SoftwareSerial(7, 8)            */
// #define camera_ON    /* camera/serial module on SoftwareSerial(12, 10)      */
// #define Husky_ON     /* DFRobot HUSKYLENS over I2C                          */
// #define oled_OFF     /* build without the OLED display                      */
// #define Servo1_OFF   /* disable the gripper on pin 9  (enabled by default)  */
// #define Servo2_OFF   /* disable the gripper on pin 10 (enabled by default)  */

#endif /* KILAT_CONFIG_H */
