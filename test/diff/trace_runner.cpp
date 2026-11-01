/*
 * trace_runner.cpp - differential test driver.
 *
 * Built twice per robot type: once against the ORIGINAL library
 * (test/golden/legacy) and once against the new one (src/).  Both builds run the
 * same deterministic random scenarios through the public API and print one
 * trace line per scenario (hash of every output write with its timestamp, plus
 * the resulting state).  `make diff` compares the two outputs: they must be
 * identical.
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdexcept>
#include "KilatTracer.h"

static uint32_t g_rng = 12345u;
static uint32_t rnd(void) { g_rng ^= g_rng << 13; g_rng ^= g_rng >> 17; g_rng ^= g_rng << 5; return g_rng; }
static uint32_t rnd_n(uint32_t n) { return rnd() % n; }

static uint16_t g_pattern = 0;
static uint8_t  g_ch_value[32];
static unsigned g_cycle = 0, g_limit = 4000;
static int g_mode = 0;                         /* 0 sensors scripted, 1 raw random   */
static uint16_t (*g_script)(unsigned) = 0;
static bool g_release_up_after = false;
static unsigned long g_release_at = 0;

/* Independent copy of the wiring truth from the ORIGINAL source (mux select
 * byte per logical channel).  Using it here, instead of the library's own
 * accessor, also checks that the new tables equal the original ones. */
#if defined(HammerHead_5) || defined(HammerHead_11)
static const uint8_t kTable[32] = {
  0x10, 0x13, 0x14, 0x16, 0x17, 0x15, 0x25, 0x27, 0x26, 0x24, 0x11, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0x20, 0x21, 0x22, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
#else
static const uint8_t kTable[32] = {
  0x04, 0x06, 0x07, 0x05, 0x02, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF, 0xFF, 0x03, 0x00, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
#endif

static int channel_of(uint8_t pin) {
  uint8_t sel = (uint8_t)(stub_digital_state(KILAT_Mux_sel[0]) | (stub_digital_state(KILAT_Mux_sel[1]) << 1) |
                          (stub_digital_state(KILAT_Mux_sel[2]) << 2));
  for (uint8_t c = 0; c < KILAT_MUX_TABLE_SIZE; c++) {
    uint8_t b = kTable[c];
    if (b == 0xFF || (b & 7) != sel) continue;
    if (KILAT_Mux_read[(b & 0x20) ? 0 : 1] != pin) continue;
    return c;
  }
  return -1;
}
static int to_raw(uint8_t v8) { long r = (long)v8 * 1023 / 255 + 1; return (int)(r > 1023 ? 1023 : r); }

static int adc_hook(uint8_t pin) {
  int ch = channel_of(pin);
  if (ch < 0) return 0;
  if (g_release_up_after && stub_adc_reads >= g_release_at) g_ch_value[MUX_CH_UP_DOWN] = 255;
  if (ch == KILAT_IRnumber[0] && g_script) {
    g_cycle++;
    if (g_cycle > g_limit) throw std::runtime_error("limit");
    g_pattern = g_script(g_cycle);
  }
  for (uint8_t i = 0; i < KILAT_IR_COUNT; i++)
    if (KILAT_IRnumber[i] == ch) {
      if (g_mode == 1) return to_raw((uint8_t)(rnd() & 0xFF));
      return to_raw(((g_pattern >> i) & 1u) ? 200 : 10);
    }
  return to_raw(g_ch_value[ch]);
}

static void world(void) {
  stub_reset(); stub_trace_reset(); stub_set_adc_hook(adc_hook);
  for (int i = 0; i < 32; i++) g_ch_value[i] = 255;
  g_ch_value[MUX_CH_BATT_MON] = 200;
  for (uint8_t i = 0; i < KILAT_EEPROM_BYTES; i++) g_IR_threshold[i] = 100;
  g_cycle = 0; g_limit = 4000; g_mode = 0; g_script = 0; g_release_up_after = false;
}

/* ---- pattern generation: benign (never a junction), junction-like, lost --- */
static uint16_t benign(void) {
  const uint8_t n = KILAT_IR_COUNT;
  uint8_t start = (uint8_t)rnd_n(n);
  uint8_t width = (uint8_t)(1 + rnd_n(KILAT_IR_COUNT > 5 ? 3 : 2));
  uint16_t w = 0;
  for (uint8_t i = 0; i < width && start + i < n; i++) w |= (uint16_t)(1u << (start + i));
  return w;
}
static uint16_t rand_word(void) { return (uint16_t)(rnd() & KILAT_IR_MASK); }
static uint16_t from5(uint8_t p5) {
#if defined(HammerHead_11)
  uint16_t w = 0;
  if (p5 & 0b10000) w |= (uint16_t)(1u << (9 + rnd_n(2)));
  if (p5 & 0b01000) w |= (uint16_t)(1u << (7 + rnd_n(2)));
  if (p5 & 0b00100) w |= (uint16_t)(1u << (4 + rnd_n(3)));
  if (p5 & 0b00010) w |= (uint16_t)(1u << (2 + rnd_n(2)));
  if (p5 & 0b00001) w |= (uint16_t)(1u << rnd_n(2));
  return w;
#else
  return p5;
#endif
}
static const uint8_t kJ[] = { 0b10010, 0b01001, 0b10001, 0b10101, 0b01110, 0b10100, 0b00101, 0b11111, 0b10110 };

static unsigned g_benign_until, g_lost_from, g_lost_to;
static uint16_t script_mix(unsigned c) {
  if (c >= g_lost_from && c < g_lost_to) return 0;
  if (c <= g_benign_until) return benign();
  uint32_t r = rnd_n(10);
  if (r < 4) return from5(kJ[rnd_n(sizeof kJ)]);
  if (r < 5) return 0;
  if (r < 7) return rand_word();
  return benign();
}

static void print_line(const char *kind, int n, const char *outcome) {
  printf("%s %d hash=%016llx t=%u lastIR=%u turn=%u out=%s\n", kind, n,
         (unsigned long long)stub_trace_hash(), (unsigned)millis(), (unsigned)last_IR,
         (unsigned)turnType, outcome);
}

static void scenario_junction(int n) {
  world();
  static const char *types[] = { "Y", "T", "LEFT", "RIGHT", "SKIP" };
  static const uint16_t speeds[] = { 25, 40, 50, 75, 100, 120, 150, 160, 200 };
  static const float kps[] = { 0.2f, 0.4f, 0.8f, 1.5f };
  static const float kds[] = { 0.0f, 0.1f, 0.2f, 0.6f };
  static const int16_t skips[] = { 0, 10, 40, 120, -1 };
  const char *type = types[rnd_n(5)];
  uint16_t spd = speeds[rnd_n(9)];
#if defined(HammerHead_11)
  /* the original overshoots an odd-parity target by one when ramping by 2 (see
   * CHANGELOG, "speed ramp"): stay on targets that the ramp reaches exactly. */
  if (spd > 150 && ((spd - 150) & 1)) spd++;
#endif
  const float kp = kps[rnd_n(4)], kd = kds[rnd_n(4)];
  const int16_t skip = skips[rnd_n(5)];
  const bool gap = rnd_n(3) == 0;
  const bool lc = rnd_n(4) == 0;
  g_benign_until = (unsigned)rnd_n(40);
  g_lost_from = (unsigned)rnd_n(30); g_lost_to = g_lost_from + (unsigned)rnd_n(8);
  g_script = script_mix;
  last_IR = (uint8_t)(rnd_n(2) ? 0b10 : 0b01);
  turnType = (uint8_t)(1 + rnd_n(3));
  g_limit = 600;
  const char *out = "returned";
  try { junction(type, spd, kp, kd, skip, gap, lc); } catch (const std::runtime_error &) { out = "limit"; }
  print_line("junction", n, out);
}

static uint16_t script_turn_end(unsigned c) {
  if (c < g_benign_until) return benign();
  return rand_word();
}
static void scenario_action(int n) {
  world();
  static const char *dirs[] = { "LEFT", "RIGHT", "STOP", "LEFT", "RIGHT" };
  static const uint8_t speeds[] = { 0, 0, 10, 30, 60, 100, 150, 200 };
  const char *dir = dirs[rnd_n(5)];
  const uint8_t spd = speeds[rnd_n(8)];
  const uint16_t dur = (uint16_t)(rnd_n(3) == 0 ? (50 + rnd_n(200)) : 0);
  const bool lc = rnd_n(4) == 0;
  g_benign_until = (unsigned)rnd_n(25);
  g_script = script_turn_end;
  turnType = (uint8_t)(rnd_n(4));          /* 0..3, including unknown values */
  last_IR = (uint8_t)(rnd_n(2) ? 0b10 : 0b01);
  g_limit = 300;
  const char *out = "returned";
  try { action(dir, spd, dur, lc); } catch (const std::runtime_error &) { out = "limit"; }
  print_line("action", n, out);
}

static void scenario_pid(int n) {
  world();
  const int steps = 20 + (int)rnd_n(40);
  for (int i = 0; i < steps; i++) {
    delay(1 + rnd_n(20));
    const uint8_t spd = (uint8_t)(25 + rnd_n(150));
    const int16_t err = (int16_t)((int)rnd_n(1001) - 500);
    const float kp = (float)rnd_n(200) / 100.0f, kd = (float)rnd_n(100) / 50.0f;
    KILAT_PID(spd, err, kp, kd);
  }
  print_line("pid", n, "done");
}

static void scenario_buttons(int n) {
  world();
  unsigned acc = 0;
  for (int i = 0; i < 30; i++) {
    g_ch_value[MUX_CH_UP_DOWN] = (uint8_t)rnd_n(256);
    g_ch_value[MUX_CH_START_BTN] = (uint8_t)rnd_n(256);
    acc = acc * 7 + KILAT_getButton();
  }
  printf("buttons %d acc=%u hash=%016llx\n", n, acc, (unsigned long long)stub_trace_hash());
}

static void scenario_ir(int n) {
  world();
  unsigned long acc = 0;
  for (int i = 0; i < 40; i++) {
    g_pattern = rand_word();
    for (uint8_t k = 0; k < KILAT_IR_COUNT; k++) g_IR_threshold[k] = (uint8_t)(20 + rnd_n(180));
    g_mode = (i & 1);                       /* alternate scripted / fully random readings */
    g_script = 0;
    acc = acc * 31 + KILAT_IRupdate(false) + 7 * KILAT_IRupdate(true);
  }
  printf("ir %d acc=%lu\n", n, acc);
}

static void scenario_calibrate(int n) {
  world();
  g_mode = 1;                               /* random readings */
  g_ch_value[MUX_CH_UP_DOWN] = 150;         /* UP held ... */
  g_release_up_after = true;
  g_release_at = 80 + rnd_n(400);           /* ... released after some reads */
  g_script = 0;
  /* keep the readings in a range where 3 x min cannot overflow a uint8_t, the
   * one documented difference (CHANGELOG, "threshold saturates"). */
  g_rng ^= 0x9E3779B9u;
  KILAT_IRcalibrate();
  printf("calibrate %d hash=%016llx thr=", n, (unsigned long long)stub_trace_hash());
  for (uint8_t i = 0; i < KILAT_IR_COUNT; i++) printf("%u,", (unsigned)g_IR_threshold[i]);
  printf(" eeprom=");
  for (uint8_t i = 0; i < KILAT_IR_COUNT; i++) printf("%u,", (unsigned)EEPROM.read(i));
  printf("\n");
}

int main(int argc, char **argv) {
  const int N = (argc > 1) ? atoi(argv[1]) : 1500;
  g_rng = (argc > 2) ? (uint32_t)atoi(argv[2]) : 12345u;
  for (int i = 0; i < N; i++) {
    scenario_junction(i);
    scenario_action(i);
    if (i % 5 == 0) scenario_pid(i);
    if (i % 10 == 0) { scenario_buttons(i); scenario_ir(i); }
    if (i % 25 == 0) scenario_calibrate(i);
  }
  return 0;
}
