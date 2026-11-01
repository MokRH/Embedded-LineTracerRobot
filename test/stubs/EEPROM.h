#ifndef EEPROM_STUB_H
#define EEPROM_STUB_H
#include <stdint.h>
struct EEPROMClass {
  uint8_t data[1024];
  uint8_t read(int i) { return data[i]; }
  void write(int i, uint8_t v) { data[i] = v; }
};
extern EEPROMClass EEPROM;
#endif
