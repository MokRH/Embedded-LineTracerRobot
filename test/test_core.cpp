/*
 * test_core.cpp - unit tests for the hardware-independent logic (KilatCore).
 *
 * Expected values were derived by hand from the original firmware logic, not
 * copied from the new implementation, so the tests act as characterisation
 * tests: they pin down the behaviour of the original code.  Tests that cover
 * deliberate fixes are marked "FIX".
 */
#include "minitest.h"
#include "../src/KilatCore.h"

using namespace kilat;

static const Profile *const kAll[4] = { &kHammerHead5, &kHammerHead11, &kComel2, &kComel3 };

/* -------------------------------------------------------------------------- */
/* Profiles                                                                   */
/* -------------------------------------------------------------------------- */
TEST(profiles_are_internally_consistent) {
  for (int i = 0; i < 4; i++) {
    const Profile &p = *kAll[i];
    CHECK_EQ(p.irMask, (1u << p.irCount) - 1u);
    CHECK(p.irCenter < p.irCount);
    CHECK(p.spdLo <= p.spdHi);
    CHECK(p.spdTargetMin >= 1);
    CHECK(p.actSpdLo > 0);
    CHECK(p.traceLo < p.traceHi);
    CHECK(p.turnLeftLoBit < p.irCount && p.turnLeftHiBit < p.irCount);
    CHECK(p.turnRightLoBit < p.irCount && p.turnRightHiBit < p.irCount);
    CHECK(p.spdUpMs >= 1 && p.spdUpStep >= 1);
    if (p.hasLineAggregation) CHECK_EQ(p.irCount, 11);
  }
}

TEST(profile_values_match_the_original_header) {
  CHECK_EQ(kHammerHead5.irCount, 5);   CHECK_EQ(kHammerHead5.irCenter, 2);
  CHECK_EQ(kHammerHead11.irCount, 11); CHECK_EQ(kHammerHead11.irCenter, 5);
  CHECK_EQ(kComel2.turnT, 1);          CHECK_EQ(kComel2.turnSide, 2);
  CHECK_EQ(kHammerHead5.turnT, 3);     CHECK_EQ(kHammerHead5.turnSide, 3);
  CHECK(!kComel3.hasTurnType3 && kHammerHead11.hasTurnType3);
  CHECK_EQ(kComel2.thresholdMode, THRESHOLD_MID_X1_04);
  CHECK_EQ(kComel3.thresholdMode, THRESHOLD_MIN_X3);
}

TEST(arduino_map_matches_the_classic_formula) {
  CHECK_EQ(arduinoMap(512, 0, 1023, 0, 255), 127);
  CHECK_EQ(arduinoMap(0, 0, 1023, 0, 255), 0);
  CHECK_EQ(arduinoMap(1023, 0, 1023, 0, 255), 255);
  CHECK_EQ(arduinoMap(25, 50, 255, 100, 200), 88);    /* extrapolates, truncates toward 0 */
  CHECK_EQ(arduinoMap(75, 25, 150, 150, 50), 110);
}

/* -------------------------------------------------------------------------- */
/* Sensors                                                                    */
/* -------------------------------------------------------------------------- */
TEST(irWord_black_line_sets_bits_above_threshold) {
  const uint8_t v[5] = { 10, 200, 10, 10, 10 };
  const uint8_t t[5] = { 100, 100, 100, 100, 100 };
  CHECK_EQ(irWord(kHammerHead5, v, t, false), 0b00010);
}

TEST(irWord_white_line_sets_bits_below_threshold) {
  const uint8_t v[5] = { 10, 200, 200, 200, 200 };
  const uint8_t t[5] = { 100, 100, 100, 100, 100 };
  CHECK_EQ(irWord(kHammerHead5, v, t, true), 0b00001);
}

TEST(irWord_threshold_comparison_is_strict) {
  const uint8_t v[5] = { 100, 100, 100, 100, 100 };
  const uint8_t t[5] = { 100, 100, 100, 100, 100 };
  CHECK_EQ(irWord(kHammerHead5, v, t, false), 0);
  CHECK_EQ(irWord(kHammerHead5, v, t, true), 0);
}

TEST(irWord_uses_individual_thresholds_and_all_11_sensors) {
  uint8_t v[11], t[11];
  for (int i = 0; i < 11; i++) { v[i] = 150; t[i] = (uint8_t)(i * 20); }
  /* sensor i is on when 150 > i*20, i.e. i <= 7 */
  CHECK_EQ(irWord(kHammerHead11, v, t, false), 0x00FF);
}

TEST(aggregatePattern_folds_11_sensors_to_5) {
  CHECK_EQ(aggregatePattern(0), 0);
  CHECK_EQ(aggregatePattern(1u << 10), 0b10000);
  CHECK_EQ(aggregatePattern(1u << 9),  0b10000);
  CHECK_EQ(aggregatePattern(1u << 8),  0b01000);
  CHECK_EQ(aggregatePattern(1u << 7),  0b01000);
  CHECK_EQ(aggregatePattern(1u << 6),  0b00100);
  CHECK_EQ(aggregatePattern(1u << 5),  0b00100);
  CHECK_EQ(aggregatePattern(1u << 4),  0b00100);
  CHECK_EQ(aggregatePattern(1u << 3),  0b00010);
  CHECK_EQ(aggregatePattern(1u << 2),  0b00010);
  CHECK_EQ(aggregatePattern(1u << 1),  0b00001);
  CHECK_EQ(aggregatePattern(1u << 0),  0b00001);
  CHECK_EQ(aggregatePattern(0x07FF),   0b11111);
}

TEST(junctionPattern_depends_on_profile) {
  CHECK_EQ(junctionPattern(kHammerHead5, 0b10010), 0b10010);
  CHECK_EQ(junctionPattern(kHammerHead11, (1u << 10) | (1u << 6) | 1u), 0b10101);
}

TEST(computeError_is_zero_when_centred_or_blind) {
  CHECK_EQ(computeError(kHammerHead5, 0), 0);
  CHECK_EQ(computeError(kHammerHead5, 0b00100), 0);
  CHECK_EQ(computeError(kHammerHead5, 0b01110), 0);
  CHECK_EQ(computeError(kHammerHead11, 1u << 5), 0);
}

TEST(computeError_sign_convention) {
  /* low sensor index = right side = positive error */
  CHECK_EQ(computeError(kHammerHead5, 0b00001), 200);
  CHECK_EQ(computeError(kHammerHead5, 0b00010), 100);
  CHECK_EQ(computeError(kHammerHead5, 0b01000), -100);
  CHECK_EQ(computeError(kHammerHead5, 0b10000), -200);
  CHECK_EQ(computeError(kHammerHead11, 1u << 0), 500);
  CHECK_EQ(computeError(kHammerHead11, 1u << 10), -500);
}

TEST(computeError_averages_active_sensors) {
  CHECK_EQ(computeError(kHammerHead5, 0b00110), 50);     /* (100 + 0) / 2            */
  CHECK_EQ(computeError(kHammerHead5, 0b00011), 150);    /* (200 + 100) / 2           */
  CHECK_EQ(computeError(kHammerHead5, 0b11000), -150);
}

TEST(updateLastIR_follows_the_outer_sensors) {
  CHECK_EQ(updateLastIR(kHammerHead5, 0b10000, 0b01), 0b10);
  CHECK_EQ(updateLastIR(kHammerHead5, 0b00001, 0b10), 0b01);
  CHECK_EQ(updateLastIR(kHammerHead5, 0b10001, 0b10), 0b10);  /* both: keep */
  CHECK_EQ(updateLastIR(kHammerHead5, 0b00100, 0b01), 0b01);  /* none: keep */
}

TEST(updateLastIR_hammerhead11_uses_the_aggregated_pattern) {
  CHECK_EQ(updateLastIR(kHammerHead11, 1u << 9, 0b01), 0b10);   /* left_2  */
  CHECK_EQ(updateLastIR(kHammerHead11, 1u << 1, 0b10), 0b01);   /* right_2 */
  CHECK_EQ(updateLastIR(kHammerHead11, 1u << 8, 0b01), 0b01);   /* left_1 only: keep */
}

/* -------------------------------------------------------------------------- */
/* Junction patterns                                                          */
/* -------------------------------------------------------------------------- */
TEST(y_junction_patterns) {
  CHECK(isYJunction(kHammerHead5, 0b10010));
  CHECK(isYJunction(kHammerHead5, 0b01001));
  CHECK(!isYJunction(kHammerHead5, 0b10001));   /* H5 only has two Y patterns */
  CHECK(isYJunction(kComel2, 0b10001));
  CHECK(isYJunction(kHammerHead11, 0b10001));
  CHECK(!isYJunction(kHammerHead5, 0b00100));
}

TEST(t_junction_patterns) {
  CHECK(isTJunction(kHammerHead5, 0b10101));
  CHECK(!isTJunction(kHammerHead5, 0b01110));   /* H5: second pattern disabled */
  CHECK(isTJunction(kComel2, 0b01110));
  CHECK(isTJunction(kHammerHead11, 0b01110));
  CHECK(!isTJunction(kComel2, 0b00100));
}

TEST(left_and_right_junction_patterns) {
  CHECK(isLeftJunction(0b10100));
  CHECK(!isLeftJunction(0b10101));
  CHECK(isRightJunction(0b00101));
  CHECK(!isRightJunction(0b10101));
  CHECK(!isLeftJunction(0b00100) && !isRightJunction(0b00100));
}

/* -------------------------------------------------------------------------- */
/* PID                                                                        */
/* -------------------------------------------------------------------------- */
TEST(pid_centred_drives_straight) {
  PidState s = { 0, 0 };
  MotorPair m = pidStep(s, 100, 0, 0.4f, 0.2f, 1000);
  CHECK_EQ(m.left, 100);
  CHECK_EQ(m.right, 100);
}

TEST(pid_turns_toward_the_line) {
  PidState s = { 0, 0 };
  /* line on the right (positive error): left motor faster, right slower */
  MotorPair m = pidStep(s, 100, 100, 0.4f, 0.0f, 1000);
  CHECK_EQ(m.left, 100);          /* 140 limited to +spd */
  CHECK_EQ(m.right, 60);
  PidState s2 = { 0, 0 };
  MotorPair m2 = pidStep(s2, 100, -100, 0.4f, 0.0f, 1000);   /* line on the left */
  CHECK_EQ(m2.left, 60);
  CHECK_EQ(m2.right, 100);
}

TEST(pid_output_is_limited_to_plus_minus_spd) {
  PidState s = { 0, 0 };
  MotorPair m = pidStep(s, 50, 500, 2.0f, 0.0f, 1000);   /* control = 1000 */
  CHECK_EQ(m.left, 50);
  CHECK_EQ(m.right, -50);
}

TEST(pid_derivative_uses_the_real_KD_and_float_dt) {
  /* FIX: the original ignored KD (used KP*50) and truncated d/dt to an integer. */
  PidState s = { 0, 1000 };
  /* error jumps 0 -> 40 in 2 ms: derivative 20, KD 1.0 -> control 20 */
  MotorPair m = pidStep(s, 100, 40, 0.0f, 1.0f, 1002);
  CHECK_EQ(m.left, 100);
  CHECK_EQ(m.right, 80);
  /* sub-unit derivative must not collapse to zero: error +3 in 4 ms = 0.75/ms,
   * KD 8 -> control 6 */
  PidState s2 = { 0, 1000 };
  MotorPair m2 = pidStep(s2, 100, 3, 0.0f, 8.0f, 1004);
  CHECK_EQ(m2.right, 94);
}

TEST(pid_zero_dt_has_zero_derivative_and_no_division_by_zero) {
  PidState s = { 0, 500 };
  MotorPair m = pidStep(s, 100, 10, 0.0f, 100.0f, 500);
  CHECK_EQ(m.left, 100);
  CHECK_EQ(m.right, 100);
}

TEST(pid_remembers_the_previous_error_and_time) {
  PidState s = { 0, 0 };
  pidStep(s, 100, 25, 0.1f, 0.1f, 40);
  CHECK_EQ(s.prevError, 25);
  CHECK_EQ(s.prevTimeMs, 40u);
}

TEST(pid_survives_nan_and_huge_gains) {
  PidState s = { 0, 0 };
  float nan = 0.0f; nan = nan / nan;
  MotorPair a = pidStep(s, 100, 10, nan, 0.0f, 10);
  CHECK_EQ(a.left, 100);  CHECK_EQ(a.right, 100);
  PidState s2 = { 0, 0 };
  MotorPair b = pidStep(s2, 100, 10, 1e30f, 0.0f, 10);
  CHECK_EQ(b.left, 100);  CHECK_EQ(b.right, -100);
}

/* -------------------------------------------------------------------------- */
/* Junction state machine                                                     */
/* -------------------------------------------------------------------------- */
static StepResult step(const Profile &p, JunctionState &st, char type, int16_t skip,
                       bool gap, uint16_t ir, uint32_t now) {
  return junctionStep(p, st, type, skip, gap, ir, now);
}

TEST(junction_begin_sets_start_and_target_speeds) {
  JunctionState st;
  junctionBegin(kHammerHead5, st, 100, 123, 0b01, 77);
  CHECK_EQ(st.spd, 100); CHECK_EQ(st.spdTarget, 100);
  CHECK_EQ(st.enterTime, 123u); CHECK_EQ(st.timerSpdUp, 77u); CHECK_EQ(st.lastIR, 0b01);
  junctionBegin(kHammerHead5, st, 200, 0, 0b01, 0);
  CHECK_EQ(st.spd, 100); CHECK_EQ(st.spdTarget, 200);     /* starts at spdHi, ramps up */
  junctionBegin(kHammerHead5, st, 5, 0, 0b01, 0);
  CHECK_EQ(st.spd, 25);  CHECK_EQ(st.spdTarget, 25);      /* lower limits */
  junctionBegin(kHammerHead11, st, 10, 0, 0b01, 0);
  CHECK_EQ(st.spd, 50);  CHECK_EQ(st.spdTarget, 50);
}

TEST(junction_follows_a_centred_line) {
  JunctionState st; junctionBegin(kHammerHead5, st, 100, 0, 0b01, 0);
  StepResult r = step(kHammerHead5, st, 'Y', 0, false, 0b00100, 10);
  CHECK(r.kind == StepKind::Follow);
  CHECK_EQ(r.error, 0);
  CHECK_EQ(r.spd, 100);
}

TEST(junction_reports_the_line_offset_while_following) {
  JunctionState st; junctionBegin(kHammerHead5, st, 100, 0, 0b01, 0);
  StepResult r = step(kHammerHead5, st, 'Y', 0, false, 0b01000, 10);
  CHECK(r.kind == StepKind::Follow);
  CHECK_EQ(r.error, -100);
  CHECK_EQ(st.lastIR, 0b01);     /* only an outer sensor updates it */
}

TEST(junction_tracks_the_side_the_line_was_last_seen) {
  JunctionState st; junctionBegin(kHammerHead5, st, 100, 0, 0b01, 0);
  step(kHammerHead5, st, 'Y', 0, false, 0b10000, 1);
  CHECK_EQ(st.lastIR, 0b10);
  step(kHammerHead5, st, 'Y', 0, false, 0b00001, 2);
  CHECK_EQ(st.lastIR, 0b01);
}

TEST(junction_lost_line_traces_back_toward_the_last_side) {
  JunctionState st; junctionBegin(kHammerHead5, st, 100, 0, 0b10, 0);
  StepResult r = step(kHammerHead5, st, 'Y', 0, false, 0, 5);
  CHECK(r.kind == StepKind::Lost);
  CHECK(r.traceLeft);
  CHECK(!r.driveForward);
  CHECK_EQ(r.traceSpd, 124);     /* map(100,50,255,100,200) */
  JunctionState st2; junctionBegin(kHammerHead5, st2, 100, 0, 0b01, 0);
  CHECK(!step(kHammerHead5, st2, 'Y', 0, false, 0, 5).traceLeft);
}

TEST(junction_lost_line_with_gap_mode_drives_forward) {
  JunctionState st; junctionBegin(kHammerHead5, st, 100, 0, 0b10, 0);
  StepResult r = step(kHammerHead5, st, 'Y', 0, true, 0, 5);
  CHECK(r.kind == StepKind::Lost);
  CHECK(r.driveForward);
  CHECK_EQ(r.spd, 100);
}

TEST(junction_comel_trace_back_speed_range) {
  JunctionState st; junctionBegin(kComel2, st, 100, 0, 0b10, 0);
  CHECK_EQ(step(kComel2, st, 'Y', 0, false, 0, 5).traceSpd, 62);   /* map(100,50,255,50,100) = 12 + 50 */
}

TEST(junction_detects_y) {
  JunctionState st; junctionBegin(kHammerHead5, st, 100, 0, 0b01, 0);
  StepResult r = step(kHammerHead5, st, 'Y', 0, false, 0b10010, 10);
  CHECK(r.kind == StepKind::Detected);
  CHECK_EQ(st.turnType, 2);
}

TEST(junction_ignores_patterns_for_other_junction_types) {
  JunctionState st; junctionBegin(kHammerHead5, st, 100, 0, 0b01, 0);
  CHECK(step(kHammerHead5, st, 'L', 0, false, 0b10010, 10).kind == StepKind::Follow);
  CHECK(step(kHammerHead5, st, 'Y', 0, false, 0b10100, 11).kind == StepKind::Follow);
}

TEST(junction_detects_t_and_sets_the_profile_turn_type) {
  JunctionState a; junctionBegin(kHammerHead5, a, 100, 0, 0b01, 0);
  CHECK(step(kHammerHead5, a, 'T', 0, false, 0b10101, 10).kind == StepKind::Detected);
  CHECK_EQ(a.turnType, 3);
  JunctionState b; junctionBegin(kComel2, b, 100, 0, 0b01, 0);
  CHECK(step(kComel2, b, 'T', 0, false, 0b01110, 10).kind == StepKind::Detected);
  CHECK_EQ(b.turnType, 1);
}

TEST(junction_detects_left_and_right) {
  JunctionState l; junctionBegin(kHammerHead5, l, 100, 0, 0b01, 0);
  CHECK(step(kHammerHead5, l, 'L', 0, false, 0b10100, 10).kind == StepKind::Detected);
  CHECK_EQ(l.turnType, 3);
  JunctionState r; junctionBegin(kComel3, r, 100, 0, 0b01, 0);
  CHECK(step(kComel3, r, 'R', 0, false, 0b00101, 10).kind == StepKind::Detected);
  CHECK_EQ(r.turnType, 2);
}

TEST(junction_hammerhead11_detects_t_from_the_aggregated_pattern) {
  JunctionState st; junctionBegin(kHammerHead11, st, 100, 0, 0b01, 0);
  const uint16_t ir = (1u << 10) | (1u << 5) | (1u << 1);     /* left_2 + mid + right_2 */
  CHECK(step(kHammerHead11, st, 'T', 0, false, ir, 10).kind == StepKind::Detected);
  CHECK_EQ(st.turnType, 3);
}

TEST(junction_skip_waits_for_the_duration) {
  JunctionState st; junctionBegin(kHammerHead5, st, 100, 1000, 0b01, 0);
  CHECK(step(kHammerHead5, st, 'S', 500, false, 0b00100, 1400).kind == StepKind::Follow);
  StepResult r = step(kHammerHead5, st, 'S', 500, false, 0b00100, 1500);
  CHECK(r.kind == StepKind::Detected);
  CHECK_EQ(st.turnType, 1);
}

TEST(junction_skip_zero_ends_immediately) {
  JunctionState st; junctionBegin(kComel2, st, 100, 0, 0b01, 0);
  CHECK(step(kComel2, st, 'S', 0, false, 0b00100, 0).kind == StepKind::Detected);
  CHECK_EQ(st.turnType, 1);
}

TEST(junction_skip_minus_one_never_ends_it_keeps_following) {
  /* (uint32_t)(int16_t)-1 is 0xFFFFFFFF, so the SKIP condition never holds. */
  for (int k = 0; k < 4; k++) {
    const Profile &p = *kAll[k];
    JunctionState st; junctionBegin(p, st, 100, 0, 0b01, 0);
    StepResult r = step(p, st, 'S', -1, false, 0b00100, 100000);
    CHECK(r.kind == StepKind::Follow);
    CHECK_EQ(st.turnType, 0);
    CHECK(step(p, st, 'S', -5, false, 0b00100, 100000).kind == StepKind::Follow);
  }
}

TEST(junction_unknown_type_just_follows) {
  JunctionState st; junctionBegin(kHammerHead5, st, 100, 0, 0b01, 0);
  CHECK(step(kHammerHead5, st, 'X', 0, false, 0b10101, 10).kind == StepKind::Follow);
}

TEST(junction_speed_ramps_up_at_the_profile_rate) {
  JunctionState st; junctionBegin(kHammerHead5, st, 200, 0, 0b01, 0);   /* start 100, target 200 */
  CHECK_EQ(step(kHammerHead5, st, 'Y', 0, false, 0b00100, 1).spd, 100); /* < 2 ms */
  CHECK_EQ(step(kHammerHead5, st, 'Y', 0, false, 0b00100, 2).spd, 101);
  CHECK_EQ(step(kHammerHead5, st, 'Y', 0, false, 0b00100, 3).spd, 101);
  CHECK_EQ(step(kHammerHead5, st, 'Y', 0, false, 0b00100, 4).spd, 102);
}

TEST(junction_speed_never_overshoots_the_target) {
  /* FIX: HammerHead_11 steps by 2; 150 + 2k can pass 255.  The original then
   * handed spd = 256 (truncated to 0) to the PID for one cycle. */
  JunctionState st; junctionBegin(kHammerHead11, st, 255, 0, 0b01, 0);
  uint16_t maxSeen = 0;
  for (uint32_t t = 1; t < 400; t++) {
    StepResult r = step(kHammerHead11, st, 'Y', 0, false, 1u << 5, t);
    if (r.spd > maxSeen) maxSeen = r.spd;
  }
  CHECK_EQ(maxSeen, 255);
  CHECK_EQ(st.spd, 255);
}

TEST(junction_hammerhead11_limits_speed_to_150_after_losing_the_line) {
  JunctionState st; junctionBegin(kHammerHead11, st, 255, 0, 0b01, 0);
  for (uint32_t t = 1; t < 100; t++) step(kHammerHead11, st, 'Y', 0, false, 1u << 5, t);
  CHECK(st.spd > 150);
  step(kHammerHead11, st, 'Y', 0, false, 0, 200);   /* lost */
  CHECK_EQ(st.spd, 150);
}

TEST(junction_scripted_run_follow_lose_recover_detect) {
  JunctionState st; junctionBegin(kHammerHead5, st, 100, 0, 0b01, 0);
  uint32_t t = 0;
  const uint16_t script[] = { 0b00100, 0b00100, 0b01000, 0b10000, 0, 0, 0b10000, 0b00100, 0b10010 };
  StepKind kinds[9];
  for (int i = 0; i < 9; i++) kinds[i] = step(kHammerHead5, st, 'Y', 0, false, script[i], ++t).kind;
  CHECK(kinds[0] == StepKind::Follow);
  CHECK(kinds[3] == StepKind::Follow);
  CHECK(kinds[4] == StepKind::Lost && kinds[5] == StepKind::Lost);
  CHECK(kinds[7] == StepKind::Follow);
  CHECK(kinds[8] == StepKind::Detected);
  CHECK_EQ(st.turnType, 2);
}

/* -------------------------------------------------------------------------- */
/* Turning                                                                    */
/* -------------------------------------------------------------------------- */
TEST(turn_type_1_pivots_at_the_default_speed) {
  TurnPlan t = planTurn(kHammerHead5, 1, 'L', 0);
  CHECK(t.valid && t.move && !t.right);
  CHECK_EQ(t.spdL, 75); CHECK_EQ(t.spdR, 75);
  CHECK_EQ(t.settleMs, 110);
}

TEST(turn_type_2_biases_to_one_side) {
  TurnPlan r = planTurn(kHammerHead5, 2, 'R', 0);
  CHECK(r.right); CHECK_EQ(r.spdL, 100); CHECK_EQ(r.spdR, 18); CHECK_EQ(r.settleMs, 90);
  TurnPlan l = planTurn(kHammerHead5, 2, 'L', 0);
  CHECK(!l.right); CHECK_EQ(l.spdL, 18); CHECK_EQ(l.spdR, 100);
}

TEST(turn_type_3_is_a_strong_bias_on_hammerheads_only) {
  TurnPlan r = planTurn(kHammerHead5, 3, 'R', 0);
  CHECK(r.move); CHECK_EQ(r.spdL, 100); CHECK_EQ(r.spdR, 37);
  TurnPlan c = planTurn(kComel2, 3, 'R', 0);
  CHECK(c.valid && !c.move);             /* unknown on Comel: motors untouched */
}

TEST(turn_requested_speed_is_limited_per_profile) {
  CHECK_EQ(planTurn(kHammerHead5, 1, 'L', 10).spdL, 25);
  CHECK_EQ(planTurn(kComel2, 1, 'L', 10).spdL, 50);
  CHECK_EQ(planTurn(kHammerHead5, 1, 'L', 250).spdL, 150);
  CHECK_EQ(planTurn(kHammerHead5, 1, 'L', 90).spdL, 90);
}

TEST(turn_comel_defaults) {
  CHECK_EQ(planTurn(kComel2, 1, 'R', 0).spdL, 60);
  TurnPlan t = planTurn(kComel2, 2, 'R', 0);
  CHECK_EQ(t.spdL, 75); CHECK_EQ(t.spdR, 12);   /* map(75,25,150,0,31) = 1550/125 = 12 */
}

TEST(turn_anything_but_left_or_right_is_not_a_turn) {
  CHECK(!planTurn(kHammerHead5, 1, 'S', 0).valid);
  CHECK(!planTurn(kHammerHead5, 1, 'F', 0).valid);
}

TEST(turn_reacquire_conditions) {
  CHECK(turnReacquired(kHammerHead5, true, 1u << 3));
  CHECK(turnReacquired(kHammerHead5, true, 1u << 4));
  CHECK(!turnReacquired(kHammerHead5, true, 1u << 0));
  CHECK(turnReacquired(kHammerHead5, false, 1u << 1));
  CHECK(!turnReacquired(kHammerHead5, false, 1u << 4));
  CHECK(turnReacquired(kHammerHead11, true, 1u << 8));
  CHECK(turnReacquired(kHammerHead11, false, 1u << 2));
  CHECK(!turnReacquired(kHammerHead11, false, 1u << 5));
}

/* -------------------------------------------------------------------------- */
/* Calibration, buttons, battery, display                                     */
/* -------------------------------------------------------------------------- */
TEST(threshold_modes) {
  CHECK_EQ(thresholdFromRange(kHammerHead5, 20, 200), 60);
  CHECK_EQ(thresholdFromRange(kComel3, 20, 200), 60);
  CHECK_EQ(thresholdFromRange(kComel2, 20, 100), 62);    /* 60 * 1.04 = 62.4 */
  CHECK_EQ(thresholdFromRange(kComel2, 100, 250), 182);  /* 175 * 1.04 = 182 */
}

TEST(threshold_saturates_instead_of_overflowing) {
  /* FIX: the original cast an out-of-range float to uint8_t (undefined). */
  CHECK_EQ(thresholdFromRange(kHammerHead5, 100, 255), 255);
  CHECK_EQ(thresholdFromRange(kHammerHead11, 255, 255), 255);
  CHECK_EQ(thresholdFromRange(kComel2, 250, 255), 255);
  CHECK_EQ(thresholdFromRange(kHammerHead5, 0, 255), 0);
}

TEST(buttons_decode_like_the_original) {
  CHECK_EQ(decodeButtons(255, 255), 0);
  CHECK_EQ(decodeButtons(50, 255), 0b001);    /* DOWN  */
  CHECK_EQ(decodeButtons(150, 255), 0b010);   /* UP    */
  CHECK_EQ(decodeButtons(255, 50), 0b100);    /* START */
  CHECK_EQ(decodeButtons(100, 255), 0);       /* bounds are exclusive */
  CHECK_EQ(decodeButtons(200, 255), 0);
  CHECK_EQ(decodeButtons(50, 50), 0b101);
}

TEST(battery_percentage_and_level) {
  CHECK_EQ(batteryPercent(100), 0);
  CHECK_EQ(batteryPercent(128), 0);
  CHECK_EQ(batteryPercent(139), 50);
  CHECK_EQ(batteryPercent(150), 100);
  CHECK_EQ(batteryPercent(255), 100);
  CHECK_EQ(batteryLevel(0), 0);
  CHECK_EQ(batteryLevel(50), 7);
  CHECK_EQ(batteryLevel(100), 14);
}

TEST(elapsed_time_is_formatted_like_the_oled_display) {
  char b[16];
  formatElapsed(0, b, sizeof b);      CHECK_STR(b, "0.00sec");
  formatElapsed(95, b, sizeof b);     CHECK_STR(b, "0.09sec");
  formatElapsed(100, b, sizeof b);    CHECK_STR(b, "0.10sec");
  formatElapsed(1234, b, sizeof b);   CHECK_STR(b, "1.23sec");
  formatElapsed(59990, b, sizeof b);  CHECK_STR(b, "59.99sec");
  formatElapsed(123456, b, sizeof b); CHECK_STR(b, "123.45sec");
}

TEST(elapsed_time_never_overruns_a_small_buffer) {
  char b[4];
  CHECK_EQ(formatElapsed(1234, b, sizeof b), 3);
  CHECK_STR(b, "1.2");
  char c[1];
  CHECK_EQ(formatElapsed(1234, c, 1), 0);
  CHECK_EQ(c[0], 0);
}

int main() { return RUN_ALL_TESTS(); }
