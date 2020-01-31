#include "DynamicSensorPayload.h"

float DynamicSensorPayload::getTemperature() {
  return this->temperature;
}
void DynamicSensorPayload::setTemperature(float temperature) {
  this->temperature = temperature;
}
float DynamicSensorPayload::getHumidity() {
  return this->humidity;
}
void DynamicSensorPayload::setHumidity(float humidity) {
  this->humidity = humidity;
}
int DynamicSensorPayload::getGas() {
  return this->gas;
}
void DynamicSensorPayload::setGas(int gas) {
  this->gas = gas;
}

bool DynamicSensorPayload::getPirState() {
  return this->pirState;
}
void DynamicSensorPayload::setPirState(bool pirState) {
  this->pirState = pirState;
}

bool DynamicSensorPayload::getRelay1State() {
  return this->relay1State;
}
void DynamicSensorPayload::setRelay1State(bool relay1State) {
  this->relay1State = relay1State;
}

bool DynamicSensorPayload::getRelay2State() {
  return this->relay2State;
}
void DynamicSensorPayload::setRelay2State(bool relay2State) {
  this->relay2State = relay2State;
}

bool DynamicSensorPayload::getRelay3State() {
  return this->relay3State;
}
void DynamicSensorPayload::setRelay3State(bool relay3State) {
  this->relay3State = relay3State;
}

String DynamicSensorPayload::strPad(String str, int len, const char *pad) {
  String strTemp = "";
  String padd = String(pad);
  for (int i = str.length() ; i < len; i++) {
    strTemp += padd;
  }
  return strTemp + str;
}

int DynamicSensorPayload::strToHex(const char* str) {
  return (int) strtol(str, 0, 16);
}

String DynamicSensorPayload::dynamicSensorPayload() {
  String tempData = String("0167") + this->strPad(String((int)(this->temperature * 10), HEX), 4, "0");
  String humData = String("0268") + this->strPad(String((int)(this->humidity * 10), HEX), 4, "0");
  String gasData = String("0402") + this->strPad(String((int)(this->gas), HEX), 4, "0");
  String pirData = String("0366") + this->strPad(String( (this->pirState ? 1 : 0), HEX), 2, "0");
  String relay1Data = String("0A01") + this->strPad(String( (this->relay1State ? 1 : 0), HEX), 2, "0");
  String relay2Data = String("0B01") + this->strPad(String( (this->relay2State ? 1 : 0), HEX), 2, "0");
  String relay3Data = String("0C01") + this->strPad(String( (this->relay3State ? 1 : 0), HEX), 2, "0");
  return tempData + humData + gasData + pirData + relay1Data + relay2Data + relay3Data;
}
