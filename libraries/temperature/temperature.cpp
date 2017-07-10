#include "temperature.h"
#include <OneWire.h>
#include <DallasTemperature.h>

Temperature::Temperature(uint8_t tempSensorPin) {
  _tempSensorPin = tempSensorPin;
  _oneWire = new OneWire(_tempSensorPin);
  _dallas = new DallasTemperature(_oneWire);
  _temp = -999;

  _dallas->begin();
  _dallas->setResolution(12);
  _dallas->setWaitForConversion(false);
  _dallas->getAddress(_addr, 0);
}

Temperature::~Temperature() {
  delete _dallas;
  delete _oneWire;
}

void Temperature::requestTempAsync() {
  _dallas->requestTemperatures();
}

uint8_t Temperature::getSensorCount() {
  return _dallas->getDeviceCount();
}

void Temperature::setProbe(uint8_t index) {
  _dallas->getAddress(_addr, index);
}

double Temperature::getTemperature() {
  unsigned long now = millis();

  // If it took more than a second to convert the temperature, assume the request is borked
  if (_dallas->isConversionAvailable(_addr) || (now - _lastConversion > 1000)) { 
    _lastConversion = now;
    _temp = _dallas->getTempC(_addr);
    this->requestTempAsync();
    return _temp;
  }

  return this->_temp;
}
