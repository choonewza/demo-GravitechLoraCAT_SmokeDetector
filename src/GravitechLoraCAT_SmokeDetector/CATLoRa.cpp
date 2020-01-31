#include "CATLoRa.h"

CATLoRa::CATLoRa(Uart *streamObject) {
  _lora = streamObject;
}

void CATLoRa::begin(int baud, DisplaySSD1306 *oled) {
  String strTemp = "";
  _oled = oled;

  _lora->begin(baud);
  while (!_lora);

  pinPeripheral(10, PIO_SERCOM);
  pinPeripheral(11, PIO_SERCOM);

  while (!_lora->available()) {
    _oled->show("---LORA CONNECTING---");
    delay(3000);
  }

  if (_lora->available()) {
    strTemp = String("Lora Available : ") + String(_lora->readString());
    _oled->show(strTemp);
  }

  delay(1000);

  loraConfig();
  loraConfigABP();

  delay(250);
  Wire.begin();

  String str1 = "";
  str1 = _lora->readString();
  Serial.println(str1);
}

void CATLoRa::setDisplay(DisplaySSD1306 *oled) {
  _oled = oled;
}

void CATLoRa::loraSendCommand(const char *s) {
  while (*s) {
    _lora->write(*s++);
  }
}

void CATLoRa::transmitter(uint8_t port, String data) {
  String strTemp = String("mac tx cnf ") + String(port) + String(" ") + data;
  loraSendCommand(strTemp.c_str());
}

void CATLoRa::transmitter(uint8_t port, const char *data) {
  transmitter(port,String(data));
}

String CATLoRa::receiver() {
  String receiveTemp = _lora->readString();
  String strTemp = String(receiveTemp);

  if (strTemp == "") {
    payloadReceived = "";
    portReceived = 0;
    return "";
  }

  Serial.print("LoRa Received ==> : ");
  Serial.println(receiveTemp);

  int firstCommandReceived = strTemp.indexOf("mac rx");
  if (firstCommandReceived > 0) {
    strTemp = strTemp.substring(firstCommandReceived + 7, strTemp.length());
    firstCommandReceived = strTemp.indexOf(" ");

    portReceived = strTemp.substring(0, firstCommandReceived).toInt();

    int endOfCommand = strTemp.indexOf("\n");

    payloadReceived = strTemp.substring(firstCommandReceived + 1, endOfCommand);
  }

  return receiveTemp;
}

String CATLoRa::loRaResponse() {
  String resReturn = "NO";
  String response = _lora->readString();
  int check = response.indexOf("Ok");
  if (check > 0) {
    resReturn =  "Ok";
  }

  check = response.indexOf("already_joined");
  if (check > 0) {
    resReturn = "already_joined";
  }

  return resReturn;
}

void CATLoRa::loraConfig() {
  String str1 = "";

  //*********************************************************************
  loraSendCommand("mac set_ch_freq 0 923200000");//0
  str1 = "";
  delay(100);
  str1 = loRaResponse();
  _oled->showOnSystemSetup("set_ch_freq 0 923200000", str1);

  //*********************************************************************
  loraSendCommand("mac set_ch_freq 1 923400000");//1
  str1 = "";
  delay(100);
  str1 = loRaResponse();
  _oled->showOnSystemSetup("set_ch_freq 1 923400000", str1);

  //*********************************************************************
  loraSendCommand("mac set_ch_freq 2 922000000");//2
  str1 = "";
  delay(100);
  str1 = loRaResponse();
  _oled->showOnSystemSetup("set_ch_freq 2 922000000", str1);

  //*********************************************************************
  loraSendCommand("mac set_ch_freq 3 922200000");//3
  str1 = "";
  delay(100);
  str1 = loRaResponse();
  _oled->showOnSystemSetup("set_ch_freq 3 922200000", str1);

  //*********************************************************************
  loraSendCommand("mac set_ch_freq 4 922400000");//4
  str1 = "";
  delay(100);
  str1 = loRaResponse();
  _oled->showOnSystemSetup("set_ch_freq 4 922400000", str1);

  //*********************************************************************
  loraSendCommand("mac set_ch_freq 5 922600000");//5
  str1 = "";
  delay(100);
  str1 = loRaResponse();
  _oled->showOnSystemSetup("set_ch_freq 5 922600000", str1);

  //*********************************************************************
  loraSendCommand("mac set_ch_freq 6 922800000");//6
  str1 = "";
  delay(100);
  str1 = loRaResponse();
  _oled->showOnSystemSetup("set_ch_freq 6 922800000", str1);

  //*********************************************************************
  loraSendCommand("mac set_ch_freq 7 923000000");//7
  str1 = "";
  delay(100);
  str1 = loRaResponse();
  _oled->showOnSystemSetup("set_ch_freq 7 923000000", str1);

  //*********************************************************************
  loraSendCommand("mac set_rx2 2 923200000");//8
  str1 = "";
  delay(100);
  str1 = loRaResponse();
  _oled->showOnSystemSetup("set_rx2 2 923200000", str1);

  //*********************************************************************
  loraSendCommand("mac save");//8
  str1 = "";
  delay(3000);
  str1 = loRaResponse();
  _oled->showOnSystemSetup("mac save", str1);
}

void CATLoRa::loraConfigABP() {
  String str1 = "";
  String strTemp = "";
  char * strBuf;
  //*********************************************************************
  strTemp = String("mac set_class ") + String(lora_class);
  strBuf = util->strToChar(strTemp);
  loraSendCommand(strBuf);
  str1 = "";
  delay(250);
  str1 = loRaResponse();
  _oled->showOnSystemSetup(String("class") + String(lora_class), str1);

  //*********************************************************************
  strTemp = String("mac set_deveui ") + String(lora_eui);
  strBuf = util->strToChar(strTemp);
  loraSendCommand(strBuf);
  str1 = "";
  delay(250);
  str1 = loRaResponse();
  _oled->showOnSystemSetup(String("deveui ") + String(lora_eui), str1);

  //*********************************************************************
  strTemp = String("mac set_devaddr ") + String(lora_address);
  strBuf = util->strToChar(strTemp);
  loraSendCommand(strBuf);
  str1 = "";
  delay(250);
  str1 = loRaResponse();
  _oled->showOnSystemSetup(String("devaddr ") + String(lora_address), str1);

  //*********************************************************************
  strTemp = String("mac set_nwkskey ") + String(lora_network_session_key);
  strBuf = util->strToChar(strTemp);
  loraSendCommand(strBuf);
  str1 = "";
  delay(250);
  str1 = loRaResponse();
  _oled->showOnSystemSetup(String("nwkskey ") + String(lora_network_session_key), str1);


  //*********************************************************************
  strTemp = String("mac set_appskey ") + String(lora_application_session_key);
  strBuf = util->strToChar(strTemp);
  loraSendCommand(strBuf);
  str1 = "";
  delay(250);
  str1 = loRaResponse();
  _oled->showOnSystemSetup(String("appskey ") + String(lora_application_session_key), str1);

  //*********************************************************************
  loraSendCommand("mac save");
  str1 = "";
  delay(3000);
  str1 = loRaResponse();
  _oled->showOnSystemSetup("mac save", str1);

  //*********************************************************************
  loraSendCommand("mac join abp");
  str1 = "";
  delay(250);
  str1 = loRaResponse();
  _oled->showOnSystemSetup("mac join abp", str1);
}
