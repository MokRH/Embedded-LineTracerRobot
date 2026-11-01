/*
 * test_hal.cpp - integration tests of the hardware layer (KilatTracer.cpp +
 * KilatCore.cpp) against the stub Arduino API.
 *
 * A scripted "virtual line sensor" answers analogRead() according to the
 * multiplexer select pins the library drives, so these tests exercise the real
 * path: mux select -> ADC -> thresholds -> bit word -> junction state machine
 * -> PD controller -> PWM outputs.  Built once per robot type (see Makefile).
 * They check wiring/glue; the numeric logic is covered by test_core.cpp.
 */
#include <stdexcept>
#include "minitest.h"
#include "../src/KilatTracer.h"

/* ---- virtual sensor ------------------------------------------------------ */
static uint16_t g_pattern = 0;          /* bit i = logical sensor i sees the line */
static uint8_t  g_ch_value[32];         /* 8-bit value returned for non-IR channels */
static unsigned g_cycle = 0;            /* counts reads of logical sensor 0        */
static unsigned g_cycle_limit = 5000;
static uint16_t (*g_script)(unsigned cycle) = 0;
static void (*g_observer)(unsigned cycle) = 0;

static int channel_of(uint8_t pin) {
  uint8_t sel = (uint8_t)(stub_digital_state(KILAT_Mux_sel[0]) |
                          (stub_digital_state(KILAT_Mux_sel[1]) << 1) |
                          (stub_digital_state(KILAT_Mux_sel[2]) << 2));
  for (uint8_t c = 0; c < KILAT_MUX_TABLE_SIZE; c++) {
    uint8_t b = KILAT_muxSelectByte(c);
    if (b == 0xFF) continue;
    if ((b & 7) != sel) continue;
    if (KILAT_Mux_read[(b & 0x20) ? 0 : 1] != pin) continue;
    return c;
  }
  return -1;
}

static int to_raw(uint8_t v8) {          /* inverse of map(raw,0,1023,0,255) */
  long raw = (long)v8 * 1023 / 255 + 1;
  return (int)(raw > 1023 ? 1023 : raw);
}

static int adc_hook(uint8_t pin) {
  int ch = channel_of(pin);
  if (ch < 0) return 0;
  if (ch == KILAT_IRnumber[0]) {         /* new scan cycle */
    g_cycle++;
    if (g_cycle > g_cycle_limit) throw std::runtime_error("runaway loop");
    if (g_script) g_pattern = g_script(g_cycle);
    if (g_observer) g_observer(g_cycle);
  }
  for (uint8_t i = 0; i < KILAT_IR_COUNT; i++)
    if (KILAT_IRnumber[i] == ch) return to_raw(((g_pattern >> i) & 1u) ? 200 : 10);
  return to_raw(g_ch_value[ch]);
}

static void setup_world(uint16_t (*script)(unsigned)) {
  stub_reset();
  stub_set_adc_hook(adc_hook);
  g_script = script; g_observer = 0; g_pattern = 0; g_cycle = 0;
  for (int i = 0; i < 32; i++) g_ch_value[i] = 255;   /* no buttons pressed */
  g_ch_value[MUX_CH_BATT_MON] = 200;
  for (uint8_t i = 0; i < KILAT_EEPROM_BYTES; i++) g_IR_threshold[i] = 100;
  turnType = 1; last_IR = 0b01;
}

/* Express a 5-bit pattern (left_2, left_1, mid, right_1, right_2 = bits 4..0)
 * in this robot's sensor bits. */
static uint16_t expand(uint8_t p5) {
#if defined(HammerHead_11)
  uint16_t w = 0;
  if (p5 & 0b10000) w |= 1u << 10;
  if (p5 & 0b01000) w |= 1u << 8;
  if (p5 & 0b00100) w |= 1u << 5;
  if (p5 & 0b00010) w |= 1u << 3;
  if (p5 & 0b00001) w |= 1u << 1;
  return w;
#else
  return p5;
#endif
}

static int pwm(uint8_t motorIdx) { return stub_analog_write_value(KILAT_Motor[motorIdx]); }

/* ---- tests --------------------------------------------------------------- */
TEST(mux_table_has_a_unique_select_pair_per_logical_channel) {
  int seen[64]; int n = 0;
  uint8_t chans[KILAT_IR_COUNT + 3];
  for (uint8_t i = 0; i < KILAT_IR_COUNT; i++) chans[i] = KILAT_IRnumber[i];
  chans[KILAT_IR_COUNT] = MUX_CH_UP_DOWN;
  chans[KILAT_IR_COUNT + 1] = MUX_CH_START_BTN;
  chans[KILAT_IR_COUNT + 2] = MUX_CH_BATT_MON;
  for (uint8_t i = 0; i < KILAT_IR_COUNT + 3; i++) {
    uint8_t b = KILAT_muxSelectByte(chans[i]);
    CHECK(b != 0xFF);
    int key = (b & 7) + 8 * ((b & 0x20) ? 0 : 1);   /* (select bits, read pin) */
    for (int k = 0; k < n; k++) CHECK(seen[k] != key);
    seen[n++] = key;
  }
  CHECK_EQ(KILAT_muxSelectByte(31), 0xFF);
  CHECK_EQ(KILAT_muxSelectByte(200), 0xFF);
}

static uint16_t script_const(unsigned) { return g_pattern; }

TEST(IRupdate_packs_the_sensor_word_for_black_and_white_lines) {
  setup_world(script_const);
  g_pattern = expand(0b10010);
  const uint16_t ir = KILAT_IRupdate(false) & KILAT_IR_MASK;
  CHECK_EQ(ir, g_pattern);
  CHECK_EQ(KILAT_IRupdate(true) & KILAT_IR_MASK, (~g_pattern) & KILAT_IR_MASK);
}

TEST(getButton_decodes_the_mux_buttons) {
  setup_world(script_const);
  CHECK_EQ(KILAT_getButton(), 0);
  g_ch_value[MUX_CH_UP_DOWN] = 150;   CHECK_EQ(KILAT_getButton(), 0b010);
  g_ch_value[MUX_CH_UP_DOWN] = 50;    CHECK_EQ(KILAT_getButton(), 0b001);
  g_ch_value[MUX_CH_UP_DOWN] = 255;
  g_ch_value[MUX_CH_START_BTN] = 50;  CHECK_EQ(KILAT_getButton(), 0b100);
}

TEST(movement_drives_the_right_pwm_pins) {
  setup_world(script_const);
  KILAT_movement("FWD", 120, 80);
  CHECK_EQ(pwm(0), 120); CHECK_EQ(pwm(2), 80);
  KILAT_movement("REV", 60, 70);
  CHECK_EQ(pwm(1), 60);  CHECK_EQ(pwm(3), 70);
  KILAT_movement("LEFT", 40, 90);                 /* A reverses, B forward */
  CHECK_EQ(pwm(1), 40);  CHECK_EQ(pwm(2), 90);
  KILAT_movement("RIGHT", 90, 40);                /* A forward, B reverses */
  CHECK_EQ(pwm(0), 90);  CHECK_EQ(pwm(3), 40);
  KILAT_movement("STOP", 0, 0);
  CHECK_EQ(pwm(0), 0); CHECK_EQ(pwm(2), 0);
  CHECK_EQ(turnType, 1);
}

static uint16_t script_y(unsigned c) { return c <= 4 ? expand(0b00100) : expand(0b10010); }

TEST(junction_follows_straight_then_returns_at_a_y_junction) {
  setup_world(script_y);
  junction("Y", 100, 0.4f, 0.2f, 0, false, false);
  CHECK_EQ(turnType, 2);
  CHECK_EQ(pwm(0), 100);                 /* centred: both motors at the cruise speed */
  CHECK_EQ(pwm(2), 100);
  CHECK(stub_analog_write_count() > 0);
}

static void obs_left_trace(unsigned c) {
  /* After the lost-line cycle (cycle 5) the robot must be turning left. */
  if (c == 6) { CHECK(pwm(1) > 0); CHECK(pwm(2) > 0); CHECK_EQ(pwm(1), pwm(2)); }
}
static uint16_t script_left_loss(unsigned c) {
  if (c <= 3) return expand(0b10000);          /* line at the far left */
  if (c <= 5) return 0;                        /* lost */
  return expand(0b10010);
}

TEST(junction_traces_back_to_the_left_after_losing_the_line_on_the_left) {
  setup_world(script_left_loss);
  g_observer = obs_left_trace;
  junction("Y", 100, 0.4f, 0.2f, 0, false, false);
  CHECK_EQ(last_IR, 0b10);
  CHECK_EQ(turnType, 2);
}

static uint16_t script_gap(unsigned c) { return c <= 2 ? expand(0b00100) : (c <= 4 ? 0 : expand(0b10010)); }
static void obs_gap(unsigned c) { if (c == 5) { CHECK_EQ(pwm(0), 100); CHECK_EQ(pwm(2), 100); } }

TEST(junction_gap_mode_drives_straight_over_a_gap) {
  setup_world(script_gap);
  g_observer = obs_gap;
  junction("Y", 100, 0.4f, 0.2f, 0, true, false);
  CHECK_EQ(turnType, 2);
}

static uint16_t script_skip(unsigned) { return expand(0b00100); }
TEST(junction_skip_returns_after_the_requested_time) {
  setup_world(script_skip);
  unsigned long t0 = millis();
  junction("S", 100, 0.4f, 0.2f, 50, false, false);
  CHECK(millis() - t0 >= 50);
  CHECK(millis() - t0 < 80);
  CHECK_EQ(turnType, 1);
}

static uint16_t script_never(unsigned) { return expand(0b00100); }
TEST(junction_with_skip_minus_one_never_returns) {
  setup_world(script_never);
  g_cycle_limit = 300;
  bool runaway = false;
  try { junction("S", 100, 0.4f, 0.2f, -1, false, false); }
  catch (const std::runtime_error &) { runaway = true; }
  CHECK(runaway);
  g_cycle_limit = 5000;
}

static void obs_turn(unsigned c) {
  if (c == 2) {                      /* mid-turn: turnType 2, left turn */
    CHECK(pwm(1) > 0);               /* motor A reversing slowly        */
    CHECK_EQ(pwm(2), KILAT_PROFILE.actSpdT2);  /* motor B forward at the profile's default */
  }
}
static uint16_t script_turn(unsigned c) {
  /* no sensor for the left side until cycle 4, then the left bit (HH: 8, else 3) */
#if defined(HammerHead_11)
  return c >= 4 ? (1u << 8) : (1u << 5);
#else
  return c >= 4 ? (1u << 3) : (1u << 2);
#endif
}

TEST(action_turns_until_the_line_is_reacquired_then_stops) {
  setup_world(script_turn);
  g_observer = obs_turn;
  turnType = 2;
  action("LEFT", 0, 0, false);
  CHECK_EQ(last_IR, 0b10);
  CHECK_EQ(pwm(0), 0);
  CHECK_EQ(pwm(2), 0);
  CHECK_EQ(turnType, 1);
}

TEST(action_with_an_unknown_direction_just_stops) {
  setup_world(script_const);
  KILAT_movement("FWD", 100, 100);
  action("STOP", 0, 0, false);
  CHECK_EQ(pwm(0), 0);
  CHECK_EQ(pwm(2), 0);
}

TEST(action_with_a_duration_turns_for_that_long) {
  setup_world(script_const);
  turnType = 1;
  unsigned long t0 = millis();
  action("RIGHT", 100, 200, false);
  CHECK(millis() - t0 >= 200);
  CHECK_EQ(pwm(0), 0);
}

TEST(pid_pwm_is_symmetric_when_centred_and_biased_when_off_centre) {
  setup_world(script_const);
  KILAT_PID(100, 0, 0.4f, 0.2f);
  CHECK_EQ(pwm(0), 100); CHECK_EQ(pwm(2), 100);
  KILAT_PID(100, -100, 0.4f, 0.0f);          /* line on the left */
  CHECK(pwm(0) < pwm(2));
}

TEST(setup_helpers_do_not_touch_unrelated_pins) {
  setup_world(script_const);
  KILAT_LED("green", 1);
  CHECK_EQ(stub_digital_state(KILAT_Digi_write[0]), 1);
  KILAT_LED("green", 0);
  CHECK_EQ(stub_digital_state(KILAT_Digi_write[0]), 0);
  KILAT_LED("purple", 1);                   /* unknown colour: ignored */
  CHECK_EQ(stub_digital_state(KILAT_Digi_write[1]), 0);
}

int main() { return RUN_ALL_TESTS(); }
