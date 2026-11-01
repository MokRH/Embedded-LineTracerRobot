#ifndef SOFTWARESERIAL_STUB_H
#define SOFTWARESERIAL_STUB_H
#include <stdint.h>
#include <stddef.h>
struct SoftwareSerial {
  SoftwareSerial(uint8_t, uint8_t) {}
  void begin(long) {}
  int available() { return 0; }
  int read() { return 0; }
  size_t write(const char *) { return 0; }
};
#endif
