#include "CommandController.h"

CommandController::~CommandController() {
  
}

CommandController::CommandController(DS3231* clock, GSensorForFloat* tHTS221, GSensorForFloat*  hHTS221, MQ2Sensor* gas) {
  this->clock = clock;
  this->tHTS221 = tHTS221;
  this->hHTS221 = hHTS221;
  this->gas = gas;
}


void CommandController::doAction(uint8_t port, String payload) {
  Serial.println(F("--############ doAction #############--"));
  Serial.print(F("doAction() - Port : "));
  Serial.println(port);
  Serial.print(F("doAction() - Payload : "));
  Serial.println(payload);

  if (port == 10) {
    doActuatorCommandsAction (port, payload);
  } else if (port == 11) {
    doDeviceConfigurationAction (port, payload);
  }
}

void CommandController::doActuatorCommandsAction(uint8_t port, String payload) {
  Serial.println(F("----10 Commands Action [START]----"));
  Serial.println();

  Serial.print(F("10 Commands Action => Payload = "));
  Serial.println(payload);

  String strTemp;
  uint8_t channel, loopCounter = 0;
  uint16_t value;

  while (payload.length() > 0) {
    Serial.print(F("10 Commands Action => Payload "));
    Serial.print(loopCounter);
    Serial.print(F(" = "));
    Serial.println(payload);
    Serial.println();

    strTemp = payload.substring(0, 2);
    channel = (uint8_t) strtol(strTemp.c_str(), 0, 16);
    payload = payload.substring(2, payload.length());

    strTemp = payload.substring(0, 4);
    value = (uint8_t) strtol(strTemp.c_str(), 0, 16);
    payload = payload.substring(4, payload.length());

    if (++loopCounter > 100) {
      Serial.println(F("-------ERROR LOOP-------"));
      break;
    }
  }

  Serial.println();
  Serial.println(F("----10 Commands Action [END]----"));
  Serial.println();
}

void CommandController::doDeviceConfigurationAction (uint8_t port, String payload) {
  //1 bit is configMask
  uint8_t configMask = payload.substring(0, 2).toInt();
  payload = payload.substring(2, payload.length());

  if ( (configMask & 1) != 0 ) {
    payload = utcTimeConfig(payload);
  }

  if ( (configMask & 2) != 0 ) {
    payload = txPeriodConfig(payload);
  }

  if ( (configMask & 4) != 0 ) {
    payload = rxPeriodConfig(payload);
  }

  if ( (configMask & 8) != 0 ) {
    payload = scheduleConfig(payload);
  }
}

String CommandController::utcTimeConfig(String payload) {
  Serial.println(F("----00000001 Set UTC TIME [START]----"));
  Serial.println();

  Serial.print(F("00000001 => Payload = "));
  Serial.println(payload);

  //4 bit is utc timestamp
  String strTemp = payload.substring(0, 8);
  uint32_t utcTime = (uint32_t) strtol(strTemp.c_str(), 0, 16);
  clock->setDateTime(utcTime - ADJUST_TIMEZONE);

  Serial.print(F("[UtcTimestampPayload, UtcTimestamp] = ["));
  Serial.print(strTemp);
  Serial.print(F(", "));
  Serial.print(utcTime);
  Serial.println("]");

  payload = payload.substring(8, payload.length());

  Serial.println();
  Serial.println(F("----00000001 Set UTC TIME [END]----"));
  Serial.println();

  return payload;
}



String CommandController::txPeriodConfig(String payload) {
  Serial.println("----00000100 Set TX Period [START]----");
  Serial.println("----00000010 Set TX Period [END]----");
  Serial.println();
  return payload;
}

String CommandController::rxPeriodConfig(String payload) {
  Serial.println("----00000100 Set RX Period [START]----");
  Serial.println("----00000100 Set RX Period [END]----");
  Serial.println();
  return payload;
}

String CommandController::scheduleConfig(String payload) {
  Serial.println("----00001000 Set Device Config [START]----");
  Serial.println();

  String strTemp;
  uint8_t channel, type, dayOfWeekRepeat, loopCounter = 0;
  uint32_t startTime, endTime;


  while (payload.length() > 0) {
    Serial.print("00001000 => Payload ");
    Serial.print(loopCounter);
    Serial.print(" = ");
    Serial.println(payload);
    Serial.println();

    // 1 bit for chanel
    strTemp = payload.substring(0, 2);
    channel = (uint8_t) strtol(strTemp.c_str(), 0, 16);
    payload = payload.substring(2, payload.length());

    strTemp = payload.substring(0, 2);
    type = (uint8_t) strtol(strTemp.c_str(), 0, 16);
    payload = payload.substring(2, payload.length());

    if (type == 1) {
      payload = deviceScheduleConfig(channel, type, payload);
    } else if (type == 2) {
      payload = moreThenSensorControlDevicesScheduleConfig(channel, type, payload);
    } else if (type == 3) {
      payload = lessThenSensorControlDevicesScheduleConfig(channel, type, payload);
    } else if (type == 4) {
      payload = betweenSensorControlDevicesScheduleConfig(channel, type, payload);
    } else if (type == 5) {
      payload = notBetweenSensorControlDevicesScheduleConfig(channel, type, payload);
    } else if (type == 6) {
      payload = removeScheduleConfig(channel, type, payload);
    } else {
      Serial.println(F("======ERROR [START]======"));
      Serial.println(channel);
      Serial.println(type);
      Serial.println(payload);
      Serial.println(F("======ERROR [END]======"));
      break;
    }

    if (++loopCounter > 100) {
      Serial.println(F("<<<<<< LOOP ERROR >>>>>>"));
      break;
    }

  } // while loop

  Serial.println(F("----00001000 Set Device Config [END]----"));
  Serial.println();

  return payload;
}

String CommandController::deviceScheduleConfig( uint8_t channel, uint8_t type, String payload) {
  Serial.println("Type 01 : Device turn-on in time [START]----");
  Serial.print("Type 01 => Payload = ");
  Serial.println(payload);

  String strTemp;
  uint8_t dayOfWeekRepeat;
  uint32_t startTime, endTime;

  strTemp = payload.substring(0, 2);
  dayOfWeekRepeat = (uint8_t) strtol(strTemp.c_str(), 0, 16);
  payload = payload.substring(2, payload.length());

  strTemp = payload.substring(0, 6);
  startTime = (uint32_t) strtol(strTemp.c_str(), 0, 16);
  payload = payload.substring(6, payload.length());

  strTemp = payload.substring(0, 6);
  endTime = (uint32_t) strtol(strTemp.c_str(), 0, 16);
  payload = payload.substring(6, payload.length());

  Serial.println("Type 01 : Sensor&Device turn-on in time [END]----");
  Serial.println();

  return payload;
}

String CommandController::moreThenSensorControlDevicesScheduleConfig( uint8_t channel, uint8_t type, String payload) {
  Serial.println("---Type 02 : Sensor control devices.(MORE THEN) [START]----");
  Serial.print("Type 02 => Payload = ");
  Serial.println(payload);

  String strTemp;
  uint8_t dayOfWeekRepeat, numberOfDevices, deviceAttachChannel;
  uint16_t activeValue, activeDuration;
  uint32_t startTime, endTime;

  strTemp = payload.substring(0, 2);
  dayOfWeekRepeat = (uint8_t) strtol(strTemp.c_str(), 0, 16);
  payload = payload.substring(2, payload.length());

  strTemp = payload.substring(0, 6);
  startTime = (uint32_t) strtol(strTemp.c_str(), 0, 16);
  payload = payload.substring(6, payload.length());

  strTemp = payload.substring(0, 6);
  endTime = (uint32_t) strtol(strTemp.c_str(), 0, 16);
  payload = payload.substring(6, payload.length());

  strTemp = payload.substring(0, 4);
  activeValue = (uint32_t) strtol(strTemp.c_str(), 0, 16);
  payload = payload.substring(4, payload.length());

  strTemp = payload.substring(0, 4);
  activeDuration = (uint32_t) strtol(strTemp.c_str(), 0, 16);
  payload = payload.substring(4, payload.length());

  strTemp = payload.substring(0, 2);
  numberOfDevices = (uint8_t) strtol(strTemp.c_str(), 0, 16);
  payload = payload.substring(2, payload.length());

  GDevice* gds[numberOfDevices];

  for (uint32_t i = 0; i < numberOfDevices; i++) {

    strTemp = payload.substring(0, 2);
    deviceAttachChannel = (uint8_t) strtol(strTemp.c_str(), 0, 16);
    payload = payload.substring(2, payload.length());
  }

  switch (channel) {
    case CHANNEL_TEMPERATURE :
      Serial.println("Type 02 => Temperature Setting [MORE THEN]");
      tHTS221->addMoreThenTimeTask(gds, numberOfDevices, activeDuration, dayOfWeekRepeat, startTime, endTime, (float) activeValue / 10);
      tHTS221->showTasks();
      break;
    case CHANNEL_HUMIDITY :
      Serial.println("Type 02 => Humidity Setting [MORE THEN]");
      hHTS221->addLessThenTimeTask(gds, numberOfDevices, activeDuration, dayOfWeekRepeat, startTime, endTime, (float) activeValue / 10);
      hHTS221->showTasks();
      break;
    case CHANNEL_GAS :
      Serial.println("Type 02 => GAS Setting [MORE THEN]");
      gas->addMoreThenTimeTask(gds, numberOfDevices, activeDuration, dayOfWeekRepeat, startTime, endTime, activeValue);
      gas->showTasks();
      break;
  }

  Serial.println("---Type 02 : Sensor control devices.(MORE THEN) [END]----");
  Serial.println();

  return payload;
}

String CommandController::lessThenSensorControlDevicesScheduleConfig( uint8_t channel, uint8_t type, String payload) {
  Serial.println("---Type 03 : Sensor control devices.(LESS THEN) [START]----");
  Serial.print("Type 03 => Payload = ");
  Serial.println(payload);

  String strTemp;
  uint8_t dayOfWeekRepeat, numberOfDevices, deviceAttachChannel;
  uint16_t activeValue, activeDuration;
  uint32_t startTime, endTime;

  strTemp = payload.substring(0, 2);
  dayOfWeekRepeat = (uint8_t) strtol(strTemp.c_str(), 0, 16);
  payload = payload.substring(2, payload.length());

  strTemp = payload.substring(0, 6);
  startTime = (uint32_t) strtol(strTemp.c_str(), 0, 16);
  payload = payload.substring(6, payload.length());

  strTemp = payload.substring(0, 6);
  endTime = (uint32_t) strtol(strTemp.c_str(), 0, 16);
  payload = payload.substring(6, payload.length());

  strTemp = payload.substring(0, 4);
  activeValue = (uint32_t) strtol(strTemp.c_str(), 0, 16);
  payload = payload.substring(4, payload.length());

  strTemp = payload.substring(0, 4);
  activeDuration = (uint32_t) strtol(strTemp.c_str(), 0, 16);
  payload = payload.substring(4, payload.length());

  strTemp = payload.substring(0, 2);
  numberOfDevices = (uint8_t) strtol(strTemp.c_str(), 0, 16);
  payload = payload.substring(2, payload.length());

  GDevice* gds[numberOfDevices];

  for (uint32_t i = 0; i < numberOfDevices; i++) {
    Serial.print("Type 03 => Number of Devices Payload ");
    Serial.print(i);
    Serial.print(" = ");
    Serial.println(payload);

    strTemp = payload.substring(0, 2);
    deviceAttachChannel = (uint8_t) strtol(strTemp.c_str(), 0, 16);
    payload = payload.substring(2, payload.length());
  }

  switch (channel) {
    case CHANNEL_TEMPERATURE :
      Serial.println("Type 03 => Temperature Setting [LESS THEN]");
      tHTS221->addLessThenTimeTask(gds, numberOfDevices, activeDuration, dayOfWeekRepeat, startTime, endTime, (float) activeValue / 10);
      tHTS221->showTasks();
      break;
    case CHANNEL_HUMIDITY :
      Serial.println("Type 03 => Humidity Setting [LESS THEN]");
      hHTS221->addLessThenTimeTask(gds, numberOfDevices, activeDuration, dayOfWeekRepeat, startTime, endTime, (float) activeValue / 10);
      hHTS221->showTasks();
      break;
    case CHANNEL_GAS :
      Serial.println("Type 03 => GAS Setting [LESS THEN]");
      gas->addLessThenTimeTask(gds, numberOfDevices, activeDuration, dayOfWeekRepeat, startTime, endTime, activeValue);
      gas->showTasks();
      break;
  }
  Serial.println("---Type 03 : Sensor control devices.(LESS THEN) [END]----");
  Serial.println();

  return payload;
}

String CommandController::betweenSensorControlDevicesScheduleConfig( uint8_t channel, uint8_t type, String payload) {
  Serial.println("---Type 04 : Sensor control devices.(BETWEEN) [START]----");
  Serial.print("Type 04 => Payload = ");
  Serial.println(payload);

  String strTemp;
  uint8_t dayOfWeekRepeat, numberOfDevices, deviceAttachChannel;
  uint16_t maxActiveValue, minActiveValue, activeDuration;
  uint32_t startTime, endTime;

  strTemp = payload.substring(0, 2);
  dayOfWeekRepeat = (uint8_t) strtol(strTemp.c_str(), 0, 16);
  payload = payload.substring(2, payload.length());

  strTemp = payload.substring(0, 6);
  startTime = (uint32_t) strtol(strTemp.c_str(), 0, 16);
  payload = payload.substring(6, payload.length());

  strTemp = payload.substring(0, 6);
  endTime = (uint32_t) strtol(strTemp.c_str(), 0, 16);
  payload = payload.substring(6, payload.length());

  strTemp = payload.substring(0, 4);
  minActiveValue = (uint32_t) strtol(strTemp.c_str(), 0, 16);
  payload = payload.substring(4, payload.length());

  strTemp = payload.substring(0, 4);
  maxActiveValue = (uint32_t) strtol(strTemp.c_str(), 0, 16);
  payload = payload.substring(4, payload.length());

  strTemp = payload.substring(0, 4);
  activeDuration = (uint32_t) strtol(strTemp.c_str(), 0, 16);
  payload = payload.substring(4, payload.length());

  strTemp = payload.substring(0, 2);
  numberOfDevices = (uint8_t) strtol(strTemp.c_str(), 0, 16);
  payload = payload.substring(2, payload.length());

  GDevice* gds[numberOfDevices];

  for (uint32_t i = 0; i < numberOfDevices; i++) {
    Serial.print("Type 04 => Number of Devices Payload ");
    Serial.print(i);
    Serial.print(" = ");
    Serial.println(payload);

    strTemp = payload.substring(0, 2);
    deviceAttachChannel = (uint8_t) strtol(strTemp.c_str(), 0, 16);
    payload = payload.substring(2, payload.length());
  }


  switch (channel) {
    case CHANNEL_TEMPERATURE :
      Serial.println("Type 04 => Temperature Setting [BETWEEN]");
      tHTS221->addBetweenTimeTask(gds, numberOfDevices, activeDuration, dayOfWeekRepeat, startTime, endTime, (float) minActiveValue / 10, (float) maxActiveValue / 10);
      tHTS221->showTasks();
      break;
    case CHANNEL_HUMIDITY :
      Serial.println("Type 04 => Humidity Setting [BETWEEN]");
      hHTS221->addBetweenTimeTask(gds, numberOfDevices, activeDuration, dayOfWeekRepeat, startTime, endTime, (float) minActiveValue / 10, (float) maxActiveValue / 10);
      hHTS221->showTasks();
      break;
    case CHANNEL_GAS :
      Serial.println("Type 04 => Gas Setting [BETWEEN]");
      gas->addBetweenTimeTask(gds, numberOfDevices, activeDuration, dayOfWeekRepeat, startTime, endTime, minActiveValue, maxActiveValue);
      gas->showTasks();
      break;
  }


  Serial.println("---Type 04 : Sensor control devices.(BETWEEN) [END]----");
  Serial.println();

  return payload;
}

String CommandController::notBetweenSensorControlDevicesScheduleConfig( uint8_t channel, uint8_t type, String payload) {
  Serial.println("---Type 05 : Sensor control devices.(NOT BETWEEN) [START]----");
  Serial.print("Type 05 => Payload = ");
  Serial.println(payload);

  String strTemp;
  uint8_t dayOfWeekRepeat, numberOfDevices, deviceAttachChannel;
  uint16_t maxActiveValue, minActiveValue, activeDuration;
  uint32_t startTime, endTime;

  strTemp = payload.substring(0, 2);
  dayOfWeekRepeat = (uint8_t) strtol(strTemp.c_str(), 0, 16);
  payload = payload.substring(2, payload.length());

  strTemp = payload.substring(0, 6);
  startTime = (uint32_t) strtol(strTemp.c_str(), 0, 16);
  payload = payload.substring(6, payload.length());

  strTemp = payload.substring(0, 6);
  endTime = (uint32_t) strtol(strTemp.c_str(), 0, 16);
  payload = payload.substring(6, payload.length());

  strTemp = payload.substring(0, 4);
  minActiveValue = (uint32_t) strtol(strTemp.c_str(), 0, 16);
  payload = payload.substring(4, payload.length());

  strTemp = payload.substring(0, 4);
  maxActiveValue = (uint32_t) strtol(strTemp.c_str(), 0, 16);
  payload = payload.substring(4, payload.length());

  strTemp = payload.substring(0, 4);
  activeDuration = (uint32_t) strtol(strTemp.c_str(), 0, 16);
  payload = payload.substring(4, payload.length());

  strTemp = payload.substring(0, 2);
  numberOfDevices = (uint8_t) strtol(strTemp.c_str(), 0, 16);
  payload = payload.substring(2, payload.length());

  GDevice* gds[numberOfDevices];

  for (uint32_t i = 0; i < numberOfDevices; i++) {
    Serial.print("Type 05 => Number of Devices Payload ");
    Serial.print(i);
    Serial.print(" = ");
    Serial.println(payload);

    strTemp = payload.substring(0, 2);
    deviceAttachChannel = (uint8_t) strtol(strTemp.c_str(), 0, 16);
    payload = payload.substring(2, payload.length());
  }

  switch (channel) {
    case CHANNEL_TEMPERATURE :
      Serial.println("Type 05 => Temperature Setting [NOT BETWEEN]");
      tHTS221->addNotBetweenTimeTask(gds, numberOfDevices, activeDuration, dayOfWeekRepeat, startTime, endTime, (float) minActiveValue / 10, (float) maxActiveValue / 10);
      tHTS221->showTasks();
      break;
    case CHANNEL_HUMIDITY :
      Serial.println("Type 05 => Humidity Setting [NOT BETWEEN]");
      hHTS221->addNotBetweenTimeTask(gds, numberOfDevices, activeDuration, dayOfWeekRepeat, startTime, endTime, (float) minActiveValue / 10, (float) maxActiveValue / 10);
      hHTS221->showTasks();
      break;
    case CHANNEL_GAS :
      Serial.println("Type 05 => Gas Setting [NOT BETWEEN]");
      gas->addNotBetweenTimeTask(gds, numberOfDevices, activeDuration, dayOfWeekRepeat, startTime, endTime, minActiveValue, maxActiveValue);
      gas->showTasks();
      break;
  }
  Serial.println("---Type 05 : Sensor control devices.(NOT BETWEEN) [END]----");
  Serial.println();

  return payload;
}

String CommandController::removeScheduleConfig( uint8_t channel, uint8_t type, String payload) {
  Serial.println("---Type 06 : Sensor&Device delete all task [START]----");
  Serial.print("Type 06 => Payload = ");
  Serial.println(payload);

  switch (channel) {
    case CHANNEL_TEMPERATURE :
      Serial.println("Type 06 : Remove all tasks of Temperature");
      tHTS221->removeTasks();
      break;
    case CHANNEL_HUMIDITY :
      Serial.println("Type 06 : Remove all tasks of Humidity");
      hHTS221->removeTasks();
      break;
    case CHANNEL_GAS :
      Serial.println("Type 06 : Remove all tasks of Gas");
      gas->removeTasks();
      break;
  }

  Serial.println("---Type 06 : Sensor&Device delete all task [END]----");
  Serial.println();

  return payload;
}
