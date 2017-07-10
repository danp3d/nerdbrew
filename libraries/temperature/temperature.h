#ifndef EDM_TEMPERATURE_H
#define EDM_TEMPERATURE_H

#include <inttypes.h>
#include <OneWire.h>
#include <DallasTemperature.h>

class Temperature {
private:
  // --- Pins
  uint8_t _tempSensorPin;

  // --- OneWire stuff
  OneWire* _oneWire;
  DallasTemperature* _dallas;
  DeviceAddress _addr;

  // --- Temperature buffer
  double _temp = -999;
  unsigned long _lastConversion;

public:
  Temperature(uint8_t tempSensorPin);
  ~Temperature();

  void requestTempAsync();
  uint8_t getSensorCount();
  double getTemperature();
  void setProbe(uint8_t index);
};

#endif
