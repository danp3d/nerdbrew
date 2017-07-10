#include <PCD8544.h>
#include <OneWire.h>
#include <Bounce2.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <TimerOne.h>
#include "pid_control.h"
#include "state_machine.h"
#include "temperature.h"
#include "button.h"
#include "utils.h"

// Display pins
#define CLK 7
#define DIN 8
#define DC 10
#define RST 12
#define CE 11 
PCD8544* lcd = new PCD8544(CLK, DIN, DC, RST, CE);

// PID
#define SENSOR_PIN 9
#define RELAY_PIN 13
PIDControl* pid = NULL;

// State Machine (controls the different states of the circuit)
#define ON_OFF_BTN_PIN 3
#define RIGHT_BTN_PIN 4
#define UP_BTN_PIN 5
#define DOWN_BTN_PIN 6
#define SET_BTN_PIN 2
StateMachine* stateMachine = NULL;

// Declare interrupt handler
void controlElement();

void setup() {
  // Start the LCD, PID controller and state machine
  lcd->begin(84, 48);
  lcd->setCursor(0, 0);
  lcd->clear();
  lcd->setCursor(0, 0);
  lcd->print("Nerdbrew");

  // Set pin modes
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(ON_OFF_BTN_PIN, INPUT_PULLUP);
  pinMode(RIGHT_BTN_PIN, INPUT_PULLUP);
  pinMode(UP_BTN_PIN, INPUT_PULLUP);
  pinMode(DOWN_BTN_PIN, INPUT_PULLUP);
  pinMode(SET_BTN_PIN, INPUT_PULLUP);

  digitalWrite(RELAY_PIN, LOW); // ensure it starts as low

  // Load tuninds from EEPROM
  double kp = 0.0, ki = 0.0, kd = 0.0;
  PIDControl::readTuningsFromEEPROM(kp, ki, kd);

  pid = new PIDControl(RELAY_PIN, SENSOR_PIN, 65, kp, ki, kd);
  stateMachine = new StateMachine(ON_OFF_BTN_PIN, RIGHT_BTN_PIN, UP_BTN_PIN, DOWN_BTN_PIN, SET_BTN_PIN);
  stateMachine->setPIDControl(pid);
  stateMachine->setLCD(lcd);

  Timer1.initialize(15000);
  Timer1.attachInterrupt(controlElement);
}

void loop() {
  stateMachine->process();
}

void controlElement() {
  pid->onTimer();
}
