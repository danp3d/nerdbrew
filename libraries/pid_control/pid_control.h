#ifndef EDM_PID_CONTROL_H
#define EDM_PID_CONTROL_H

#include <inttypes.h>
#include "temperature.h"
#include <Arduino.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

#define DEFAULT_DUTY_CYCLE 10000 // 10 seconds
#define DEFAULT_MIN_ON_TIME 100 // 100 milliseconds
#define DEFAULT_SAMPLE_TIME 1000 // 1 second
#define DEFAULT_KP 850
#define DEFAULT_KI 0.5
#define DEFAULT_KD 0.1
#define KP_EEPROM_ADDRESS 0
#define KI_EEPROM_ADDRESS 8
#define KD_EEPROM_ADDRESS 16 

enum PIDState {
  OFF = 0,
  ON,
  AUTOTUNE
};

class PIDControl {
private:
  // --- General configs
  uint8_t _relayPin;
  double _targetTemp; // (in PID lingo, "Set")
  double _currentTemp; // (in PID lingo, "Input")
  double _power; // (PID Output, will tell us how much power we need in a scale of 0 to _dutyCycle)
  int _dutyCycle;
  int _minOnTime; // Minimum time the heating element can stay on (prevent short bursts, which can cause problems)
  int _sampleTime;

  // --- PID libs
  double _Kp;
  double _Ki;
  double _Kd;
  unsigned long _lastPIDSample;
  int _state = 0; // 0 = off, 1 = on, 2 = autotune
  int _previousPIDState = MANUAL; // 0 = off, 1 = on, 2 = autotune
  PID* _pid;
  PID_ATune* _atune;
  void resetPID();

  // --- Time-proportional control
  volatile long _onTime = 0; // How many milliseconds to stay on for (gets copied from _power after each loop. Avoids random changes mid-cycle)
  float _percentualPower();

  // --- Temperature
  uint8_t _tempSensorPin;
  Temperature* _temperature;

  // --- States
  unsigned long _startTime = 0;
  void processTemperature();
  void processOnState();
  void processAutoTuneState();

  // --- EEPROM write
  void writeToEEPROM(int address, double value);

public:
  PIDControl(uint8_t relayPin, uint8_t tempSensorPin, double targetTemp, double Kp, double Ki, double Kd);
  ~PIDControl();

  static void readTuningsFromEEPROM(double &kp, double &ki, double &kd);
  static double readTuningFromEEPROM(int address);

  void setRelayPin(uint8_t pin);
  void setTargetTemp(double temp);
  void setDutyCycle(int dutyCycle);
  void setMinOnTime(int minOnTime);
  void setSampleTime(int sampleTime);
  void setKp(double kp);
  void setKi(double ki);
  void setKd(double kd);
  void setTunings(double kp, double ki, double kd);
  uint8_t getRelayPin();
  uint8_t getTempSensorPin();
  double getTargetTemp();
  int getDutyCycle();
  int getMinOnTime();
  int getSampleTime();
  double getKp();
  double getKi();
  double getKd();
  double getCurrentTemp();
  float getPercentualPower();
  Temperature* getTemperature();
  int getState() { return _state; }
  String getStateStr();
  String getPowerStr();

  void turnOn();
  void turnOff();
  void autoTune();
  void process();
  void onTimer();
};

#endif
