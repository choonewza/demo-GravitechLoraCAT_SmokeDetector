#include "DisplaySSD1306.h"

DisplaySSD1306::DisplaySSD1306(Adafruit_SSD1306* oled) {
  _display = oled;
}

void DisplaySSD1306::begin() {
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!_display->begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    while (true); // Don't proceed, loop forever
  }
  _display->display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  _display->clearDisplay();
}



void DisplaySSD1306::show(const char* str , uint8_t textSize) {
  Serial.println(str);
  _display->clearDisplay();
  _display->setTextSize(textSize);      // Normal 1:1 pixel scale
  _display->setTextColor(WHITE);        // Draw white text
  _display->setCursor(0, 0);            // Start at top-left corner
  _display->println(F(str));

  _display->display();
}

void DisplaySSD1306::show(String str , uint8_t textSize) {
  show(str.c_str(), textSize);
}

void DisplaySSD1306::show(const char* str) {
  show(str, 1);
}

void DisplaySSD1306::show(String str) {
  show(str, 1);
}

void DisplaySSD1306::showOnSystemSetup(const char* str) {
  Serial.println(str);

  _display->clearDisplay();
  _display->setTextSize(1);             // Normal 1:1 pixel scale
  _display->setTextColor(WHITE);        // Draw white text
  _display->setCursor(0, 0);            // Start at top-left corner
  _display->println(F("---DEVICE START---"));

  _display->setTextColor(WHITE);        // Draw white text
  _display->println(F(str));

  _display->display();
}

void DisplaySSD1306::showOnSystemSetup(String str) {
  showOnSystemSetup(str.c_str());
}

void DisplaySSD1306::showOnSystemSetup(const char* str, const char* res) {
  Serial.print(str);
  Serial.print(" --> ");
  Serial.println(res);

  _display->clearDisplay();
  _display->setTextSize(1);             // Normal 1:1 pixel scale
  _display->setTextColor(WHITE);        // Draw white text
  _display->setCursor(0, 0);            // Start at top-left corner
  _display->println(F("---DEVICE START---"));

  _display->setTextColor(WHITE);        // Draw white text
  _display->println(F(str));

  _display->print(F("--> "));
  _display->println(F(res));

  _display->display();
}

void DisplaySSD1306::showOnSystemSetup(const char* str, String res) {
  showOnSystemSetup(str, res.c_str());
}

void DisplaySSD1306::showOnSystemSetup(String str, String res) {
  showOnSystemSetup(str.c_str(), res.c_str());
}

void DisplaySSD1306::showMainUI(const char* str, float temperature, float humidity, uint16_t gas,uint32_t txCount,uint32_t rxCount){
  _display->clearDisplay();

  _display->setCursor(0, 0);

  _display->setTextSize(1);
  _display->setTextColor(WHITE);
  _display->println(F(str));

  _display->print(F("Tem:"));
  _display->print(temperature);
  _display->print(F("C"));
  _display->print(F(" Hum:"));
  _display->print(humidity);
  _display->println(F("%"));

  _display->print(F("GAS:  "));
  _display->println(gas);
  
  _display->print(F("TX:"));
  _display->print(txCount);
  _display->print(F("   RX:"));
  _display->println(rxCount);

  _display->display();
}
