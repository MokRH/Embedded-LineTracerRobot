/* Minimal Arduino API stub so the hardware layer can be compiled and exercised
 * on a PC.  Behaviour (pin states, time, ADC values) is controlled by tests via
 * the stub_* functions below.  Not for use on a real board. */
#ifndef ARDUINO_STUB_H
#define ARDUINO_STUB_H
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <string>
#include <stdio.h>

#define PROGMEM
#define pgm_read_byte(addr) (*(const uint8_t *)(addr))
#define memcpy_P memcpy

enum { LOW = 0, HIGH = 1, INPUT = 0, OUTPUT = 1 };
static const uint8_t A0 = 14, A1 = 15, A2 = 16, A3 = 17, A4 = 18, A5 = 19, A6 = 20, A7 = 21;

#define bitRead(value, bit)  (((value) >> (bit)) & 0x01)
#define bitSet(value, bit)   ((value) |= (1UL << (bit)))
#define bitWrite(value, bit, bitvalue) \
  ((bitvalue) ? bitSet(value, bit) : ((value) &= ~(1UL << (bit))))
#define constrain(amt, low, high) \
  ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

long map(long x, long in_min, long in_max, long out_min, long out_max);
void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
void analogWrite(uint8_t pin, int val);
int  analogRead(uint8_t pin);
unsigned long millis(void);
void delay(unsigned long ms);

extern uint8_t TCCR0A, TCCR0B, TCCR2A, TCCR2B;

/* ---- test controls ------------------------------------------------------ */
void    stub_reset(void);
uint8_t stub_digital_state(uint8_t pin);
int     stub_analog_write_value(uint8_t pin);
unsigned long stub_analog_write_count(void);
typedef int (*stub_adc_hook_t)(uint8_t pin);
void    stub_set_adc_hook(stub_adc_hook_t hook);
void    stub_set_time_ms(unsigned long ms);
extern unsigned long stub_adc_reads;
uint64_t stub_trace_hash(void);   /* FNV hash of every digital/analog write + time */
void     stub_trace_reset(void);


/* Minimal Arduino String: only the original (legacy) code uses it. */
class String {
 public:
  String(const char *s = "") : s_(s) {}
  String(unsigned long v) { char b[24]; snprintf(b, sizeof b, "%lu", v); s_ = b; }
  String(unsigned int v) : String((unsigned long)v) {}
  String(int v) { char b[24]; snprintf(b, sizeof b, "%d", v); s_ = b; }
  String(unsigned char v) : String((unsigned long)v) {}
  String(const std::string &s) : s_(s) {}
  String operator+(const String &o) const { return String(s_ + o.s_); }
  void toCharArray(char *buf, unsigned int n) const {
    if (n == 0) return;
    strncpy(buf, s_.c_str(), n - 1); buf[n - 1] = 0;
  }
 private:
  std::string s_;
};
inline String operator+(const char *a, const String &b) { return String(a) + b; }

#endif
