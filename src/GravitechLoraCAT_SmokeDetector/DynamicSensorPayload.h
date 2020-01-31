#ifndef DynamicSensorPayload_h
#define DynamicSensorPayload_h

#include "Arduino.h"

class DynamicSensorPayload {
  public:
    float getTemperature();
    void setTemperature(float temperature);
    
    float getHumidity();
    void setHumidity(float humidity);
    
    int getGas();
    void setGas(int gas);
    
    bool getPirState();
    void setPirState(bool pirState);

    bool getRelay1State();
    void setRelay1State(bool relay1State);

    bool getRelay2State();
    void setRelay2State(bool relay2State);

    bool getRelay3State();
    void setRelay3State(bool relay3State);
    
    String strPad(String str, int len, const char *pad);
    int strToHex(const char* str);
    String dynamicSensorPayload();

  private:
    float temperature;
    float humidity;
    int gas;

    //on=true, off=false
    bool pirState;
    bool relay1State;
    bool relay2State;
    bool relay3State;
};

#endif
