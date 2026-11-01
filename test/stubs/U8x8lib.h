#ifndef U8X8LIB_STUB_H
#define U8X8LIB_STUB_H
#include <stdint.h>
#define U8X8_PIN_NONE 255
static const uint8_t u8x8_font_lucasarts_scumm_subtitle_r_2x2_f[1] = { 0 };
struct U8X8_SSD1306_128X64_NONAME_HW_I2C {
  explicit U8X8_SSD1306_128X64_NONAME_HW_I2C(uint8_t) {}
  bool begin() { return true; }
  void setPowerSave(uint8_t) {}
  void setFont(const uint8_t *) {}
  void setFlipMode(uint8_t) {}
  void drawString(uint8_t, uint8_t, const char *) {}
  void clear() {}
  void drawTile(uint8_t, uint8_t, uint8_t, uint8_t *) {}
};
#endif
