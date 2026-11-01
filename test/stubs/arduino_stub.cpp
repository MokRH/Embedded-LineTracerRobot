#include "Arduino.h"
#include "EEPROM.h"
#include "Wire.h"

EEPROMClass EEPROM;
TwoWire Wire;

uint8_t TCCR0A, TCCR0B, TCCR2A, TCCR2B;
unsigned long stub_adc_reads = 0;

static uint8_t g_digital[64];
static int g_analog_write[64];
static unsigned long g_analog_write_count = 0;
static uint64_t g_trace = 1469598103934665603ULL;
static void trace(uint64_t a, uint64_t b, uint64_t c) {
  uint64_t v[3] = { a, b, c };
  for (int i = 0; i < 3; i++) { g_trace ^= v[i]; g_trace *= 1099511628211ULL; }
}
static unsigned long g_time_us = 0;
static stub_adc_hook_t g_hook = 0;

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t pin, uint8_t val) { g_digital[pin & 63] = val; trace(1000 + pin, val, g_time_us / 1000UL); }
void analogWrite(uint8_t pin, int val) {
  g_analog_write[pin & 63] = val;
  g_analog_write_count++;
  trace(2000 + pin, (uint64_t)(unsigned)val, g_time_us / 1000UL);
}
int analogRead(uint8_t pin) {
  stub_adc_reads++;
  g_time_us += 100;                       /* ~100 us per conversion */
  return g_hook ? g_hook(pin) : 0;
}
unsigned long millis(void) { return g_time_us / 1000UL; }
void delay(unsigned long ms) { g_time_us += ms * 1000UL; }

uint64_t stub_trace_hash(void) { return g_trace; }
void stub_trace_reset(void) { g_trace = 1469598103934665603ULL; }

void stub_reset(void) {
  memset(g_digital, 0, sizeof g_digital);
  memset(g_analog_write, 0, sizeof g_analog_write);
  g_analog_write_count = 0; g_time_us = 0; g_hook = 0; stub_adc_reads = 0;
}
uint8_t stub_digital_state(uint8_t pin) { return g_digital[pin & 63]; }
int stub_analog_write_value(uint8_t pin) { return g_analog_write[pin & 63]; }
unsigned long stub_analog_write_count(void) { return g_analog_write_count; }
void stub_set_adc_hook(stub_adc_hook_t hook) { g_hook = hook; }
void stub_set_time_ms(unsigned long ms) { g_time_us = ms * 1000UL; }
