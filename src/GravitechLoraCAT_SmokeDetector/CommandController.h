#ifndef CommandController_h
#define CommandController_h

#include "Arduino.h"
#include <DS3231.h>
#include "MQ2Sensor.h"
#include "PIRSensor.h"
#include "RelayDevice.h"
#include "GSensorForFloat.h"

#define CHANNEL_TEMPERATURE 1
#define CHANNEL_HUMIDITY    2
#define CHANNEL_GAS         4

#define ADJUST_TIMEZONE   3600 // -3600 for UCT sync time

class CommandController {
  public:
    ~CommandController();
    CommandController(DS3231* clock, GSensorForFloat* tHTS221, GSensorForFloat*  hHTS221, MQ2Sensor* gas);
    void doAction(uint8_t port, String payload);
    void doActuatorCommandsAction(uint8_t port, String payload);
    void doDeviceConfigurationAction (uint8_t port, String payload);
    String utcTimeConfig(String payload);
    String txPeriodConfig(String payload);
    String rxPeriodConfig(String payload);
    String scheduleConfig(String payload);
    String deviceScheduleConfig( uint8_t channel, uint8_t type, String payload);
    String moreThenSensorControlDevicesScheduleConfig( uint8_t channel, uint8_t type, String payload);
    String lessThenSensorControlDevicesScheduleConfig( uint8_t channel, uint8_t type, String payload);
    String betweenSensorControlDevicesScheduleConfig( uint8_t channel, uint8_t type, String payload);
    String notBetweenSensorControlDevicesScheduleConfig( uint8_t channel, uint8_t type, String payload);
    String removeScheduleConfig( uint8_t channel, uint8_t type, String payload);


  private:
    DS3231* clock;
    GSensorForFloat* tHTS221;
    GSensorForFloat* hHTS221;
    MQ2Sensor* gas;
};

#endif
