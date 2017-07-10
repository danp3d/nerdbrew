#include "pid_control.h"
#include <utils.h>
#include <inttypes.h>
#include <Temperature.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <EEPROM.h>

PIDControl::PIDControl(uint8_t relayPin, uint8_t tempSensorPin, double targetTemp, double Kp, double Ki, double Kd) {
  if (isnan(Kp))
    Kp = DEFAULT_KP;
  if (isnan(Ki))
    Ki = DEFAULT_KI;
  if (isnan(Kd))
    Kd = DEFAULT_KD;

  _relayPin = relayPin;
  _targetTemp = targetTemp;
  _currentTemp = 0;
  _power = 0;
  _dutyCycle = DEFAULT_DUTY_CYCLE;
  _minOnTime = DEFAULT_MIN_ON_TIME;
  _sampleTime = DEFAULT_SAMPLE_TIME;
  _Kp = Kp;
  _Ki = Ki;
  _Kd = Kd;
  _tempSensorPin = tempSensorPin;
  _state = PIDState::OFF;
  _lastPIDSample = 0;

  _temperature = new Temperature(_tempSensorPin);
  _temperature->requestTempAsync();

  _pid = new PID(&_currentTemp, &_power, &_targetTemp, _Kp, _Ki, _Kd, DIRECT);
  this->resetPID();
  _pid->SetMode(MANUAL); // do not start running yet

  _atune = new PID_ATune(&_currentTemp, &_power);
}

PIDControl::~PIDControl() {
  delete _atune;
  delete _pid;
  delete _temperature;
}

String PIDControl::getStateStr() {
  switch (_state) {
  case PIDState::OFF:
    return "OFF";
    break;
  case PIDState::ON:
    return "ON";
    break;
  case PIDState::AUTOTUNE:
    return "A.TUNE";
    break;
  default:
    return "UNKNOWN";
    break;
  }
}

String PIDControl::getPowerStr() {
  String str = Utils::floatToStr(getPercentualPower());
  return str;
}


void PIDControl::turnOn() {
  _temperature->requestTempAsync();
  _pid->SetMode(AUTOMATIC);
  _startTime = millis();
  _state = PIDState::ON;
}

void PIDControl::turnOff() {
  _state = PIDState::OFF;
}

void PIDControl::autoTune() {
  _previousPIDState = _pid->GetMode();
  _atune->SetNoiseBand(1);
  _atune->SetOutputStep(500);
  _atune->SetLookbackSec(20);
  _state = PIDState::AUTOTUNE;
}

void PIDControl::processTemperature() {
  double temp = _temperature->getTemperature();
  if (temp != -999)
    this->_currentTemp = temp;
}

void PIDControl::processOnState() {
  unsigned long now = millis();
  if (_lastPIDSample == 0 || (now - _lastPIDSample >= 100)) {
    _pid->Compute();
    _lastPIDSample = now;
  }

  _onTime = _power;
}

void PIDControl::processAutoTuneState() {
  if (_atune->Runtime()) {
    _Kp = _atune->GetKp();
    _Ki = _atune->GetKi();
    _Kd = _atune->GetKd();
    resetPID();
    _pid->SetMode(_previousPIDState);

    _state = PIDState::ON;
  }
}

void PIDControl::process() {
  this->processTemperature();
  this->_pid->SetTunings(_Kp, _Ki, _Kd); // set this in case the autotuning is running (will only write to eeprom if it actually changed)

  switch (this->_state) {
    case PIDState::ON:
      this->processOnState();
      break;
    case PIDState::AUTOTUNE:
      this->processAutoTuneState();
      break;
  }
}

void PIDControl::onTimer() {
  if (_state != PIDState::OFF) {
    unsigned long now = millis();
    unsigned long runningFor = now - _startTime;

    // Restart the duty cycle if needed
    if (runningFor > _dutyCycle) {
      _startTime += _dutyCycle;
      runningFor = now - _startTime;
    }

    if (_onTime > _minOnTime && _onTime > runningFor) { // Need to turn (or keep) the heating element on?
      digitalWrite(_relayPin, HIGH);
      return;
    }
  }

  // Nope, turn (or keep) it off
  digitalWrite(_relayPin, LOW);
}

float PIDControl::_percentualPower() {
  return this->_power * 100.0 / this->_dutyCycle;
}

void PIDControl::setRelayPin(uint8_t pin) {
  this->_relayPin = pin;
}

void PIDControl::setTargetTemp(double temp) {
  this->_targetTemp = temp;
}

void PIDControl::setDutyCycle(int dutyCycle) {
  this->_dutyCycle = dutyCycle;
}

void PIDControl::setMinOnTime(int minOnTime) {
  this->_minOnTime = minOnTime;
}

void PIDControl::setSampleTime(int sampleTime) {
  this->_sampleTime = sampleTime;
  this->resetPID();
}

void PIDControl::setKp(double kp) {
  this->setTunings(kp, _Ki, _Kd);
}

void PIDControl::setKi(double ki) {
  this->setTunings(_Kp, ki, _Kd);
}

void PIDControl::setKd(double kd) {
  this->setTunings(_Kp, _Ki, kd);
}

void PIDControl::setTunings(double kp, double ki, double kd) {
  if (_Kp != kp) {
    this->writeToEEPROM(KP_EEPROM_ADDRESS, kp);
  }
  if (_Ki != ki) {
    this->writeToEEPROM(KI_EEPROM_ADDRESS, ki);
  }
  if (_Kd != kd) {
    this->writeToEEPROM(KD_EEPROM_ADDRESS, kd);
  }

  this->_Kp = kp;
  this->_Ki = ki;
  this->_Kd = kd;
  this->resetPID();
}

uint8_t PIDControl::getRelayPin() {
  return this->_relayPin;
}

uint8_t PIDControl::getTempSensorPin() {
  return this->_tempSensorPin;
}

double PIDControl::getTargetTemp() {
  return this->_targetTemp;
}

int PIDControl::getDutyCycle() {
  return this->_dutyCycle;
}

int PIDControl::getMinOnTime() {
  return this->_minOnTime;
}

int PIDControl::getSampleTime() {
  return this->_sampleTime;
}

double PIDControl::getKp() {
  return this->_Kp;
}

double PIDControl::getKi() {
  return this->_Ki;
}

double PIDControl::getKd() {
  return this->_Kd;
}

double PIDControl::getCurrentTemp() {
  return this->_currentTemp;
}

float PIDControl::getPercentualPower() {
  return this->_percentualPower();
}

Temperature* PIDControl::getTemperature() {
  return this->_temperature;
}

void PIDControl::resetPID() {
  this->_pid->SetTunings(_Kp, _Ki, _Kd);
  this->_pid->SetSampleTime(this->_sampleTime);
  this->_pid->SetOutputLimits(0, this->_dutyCycle);
}

void PIDControl::writeToEEPROM(int address, double value) {
  byte *p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++) {
    EEPROM.write(address++, *p++);
  }
}

double PIDControl::readTuningFromEEPROM(int address) {
  double value = 0.0;
  byte *p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++) {
    *p++ = EEPROM.read(address++);
  }

  return value;
}

void PIDControl::readTuningsFromEEPROM(double &kp, double &ki, double &kd) {
  kp = PIDControl::readTuningFromEEPROM(KP_EEPROM_ADDRESS);
  ki = PIDControl::readTuningFromEEPROM(KI_EEPROM_ADDRESS);
  kd = PIDControl::readTuningFromEEPROM(KD_EEPROM_ADDRESS);
}

