#ifndef DisplaySSD1306_h
#define DisplaySSD1306_h

#include "Arduino.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

class DisplaySSD1306 {
  private:
    Adafruit_SSD1306 *_display;

  public:
    DisplaySSD1306(Adafruit_SSD1306* oled);
    void begin(void);
    void show(const char* str, uint8_t textSize);
    void show(String str, uint8_t textSize);
    void show(const char *str);
    void show(String str);
    void showOnSystemSetup(const char* str);
    void showOnSystemSetup(String str);
    void showOnSystemSetup(const char* str, const char* res);
    void showOnSystemSetup(const char* str, String res);
    void showOnSystemSetup(String str, String res);

    void showMainUI(const char* datetime, float temperature, float humidity, uint16_t gas,uint32_t txCount,uint32_t rxCount);
};

#endif
