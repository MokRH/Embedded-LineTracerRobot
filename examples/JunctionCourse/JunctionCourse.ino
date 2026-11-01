/*
 * JunctionCourse.ino
 * =============================================================================
 * Example sketch for the KILAT line-following robot library.
 *
 * Behaviour:
 *   1. Follow the line until a LEFT   junction is detected -> turn LEFT
 *   2. Follow the line until a RIGHT  junction is detected -> move FORWARD
 *   3. Follow the line until a MIDDLE (T) junction is found -> STOP
 *
 * "Middle junction" = T-junction (the line branches left + right + forward,
 * so the robot is at the centre of an intersection).
 *
 * Hardware:  ATmega328P (Arduino Uno / Nano) + KILAT shield
 * Robot:     choose the type in src/KilatConfig.h or with a build flag, e.g.
 *              arduino-cli compile --build-property "build.extra_flags=-DHammerHead_11" ...
 *            (#defines in this sketch are NOT seen by the library.)
 * =============================================================================
 */
#include <KilatTracer.h>

/* ---- Tunable parameters ---- */
#define CRUISE_SPEED     100    /* line-following cruise speed (25-255)          */
#define KP               0.4    /* PID proportional gain                         */
#define KD               0.2    /* PID derivative gain                           */
#define LINE_COLOR       false  /* false = black line on white background        */
#define GAP_MODE         false  /* false = trace back if the line is lost        */
#define FORWARD_CLEAR_MS 400    /* ms to drive forward past a right junction     */

void setup() {
  /* KILAT_setup() does everything: pins, PWM, OLED, IR calibration (hold UP
   * while switching on), then blocks until the START button is pressed. */
  KILAT_setup();
}

void loop() {
  /* Step 1: LEFT junction -> turn LEFT.
   * junction("LEFT", ...) follows the line until the left-branch pattern is
   * detected; action("LEFT", ...) performs the turn and waits until the line is
   * reacquired on the new path. */
  junction("LEFT", CRUISE_SPEED, KP, KD, 0, GAP_MODE, LINE_COLOR);
  action("LEFT", 0, 0, LINE_COLOR);

  /* Step 2: RIGHT junction -> move FORWARD (continue straight).
   * The robot does NOT turn right; it drives straight ahead to clear the
   * branch, then resumes line following in the next junction() call. */
  junction("RIGHT", CRUISE_SPEED, KP, KD, 0, GAP_MODE, LINE_COLOR);
  KILAT_movement("FWD", CRUISE_SPEED, CRUISE_SPEED);
  delay(FORWARD_CLEAR_MS);
  KILAT_movement("STOP", 0, 0);

  /* Step 3: MIDDLE (T) junction -> STOP.
   * oledDisplay_time(1) shows the run time and then blinks the LEDs forever,
   * so the loop does not need to halt explicitly. */
  junction("T", CRUISE_SPEED, KP, KD, 0, GAP_MODE, LINE_COLOR);
  KILAT_movement("STOP", 0, 0);
  oledDisplay_time(1);
}
