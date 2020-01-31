#ifndef CATLoRa_h
#define CATLoRa_h

#include "Arduino.h"
#include "wiring_private.h"
#include <SPI.h>
#include <Wire.h>
#include "DisplaySSD1306.h"
#include "Utility.h"

class CATLoRa {
  private:
    String lora_class = "C";
    String lora_eui = "?????????????????";
    String lora_address = "????????";
    String lora_network_session_key = "??????????????????????????????????";
    String lora_application_session_key = "??????????????????????????????????";

    Uart *_lora;

    void loraConfig();
    void loraConfigABP();
    void loraSendCommand(const char *s);
    String loRaResponse();

    DisplaySSD1306 *_oled;
    Utility *util;

  public:
    CATLoRa(Uart *streamObject);
    void begin(int baud, DisplaySSD1306 *oled);
    void setDisplay(DisplaySSD1306 *oled);

    void transmitter(uint8_t port, String data);
    void transmitter(uint8_t port, const char *data);
    String receiver();

    String payloadReceived;
    uint8_t portReceived = 0;

};

#endif
