#ifndef HUSKYLENS_STUB_H
#define HUSKYLENS_STUB_H
#include "Wire.h"
struct HUSKYLENSResult { int ID; };
struct HUSKYLENS {
  bool begin(TwoWire &) { return true; }
  bool request() { return false; }
  bool isLearned() { return false; }
  bool available() { return false; }
  HUSKYLENSResult read() { HUSKYLENSResult r = { 0 }; return r; }
};
#endif
