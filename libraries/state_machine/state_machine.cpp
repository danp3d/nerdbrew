#include "state_machine.h"
#include "button.h"
#include "utils.h"
#include <PCD8544.h>
#include <inttypes.h>

StateMachine::StateMachine(int onOffBtnPin, int rightBtnPin, int upBtnPin, int downBtnPin, int setBtnPin) {
  _currentState = AppStates::STATE_MAIN;
  _onOffExit = new Button(onOffBtnPin);
  _right = new Button(rightBtnPin);
  _up = new Button(upBtnPin);
  _down = new Button(downBtnPin);
  _set = new Button(setBtnPin);
}

StateMachine::~StateMachine() {
  delete _onOffExit;
  delete _right;
  delete _up;
  delete _down;
  delete _set;
}

void StateMachine::setPIDControl(PIDControl* pid) {
  _pid = pid;
}

void StateMachine::setLCD(PCD8544* lcd) {
  _lcd = lcd;
}

void StateMachine::process(void) {
  _pid->process();
  _onOffExit->process();
  _right->process();
  _up->process();
  _down->process();
  _set->process();

  // Pressed the 'up' button
  if (_up->pushed()) {
    _up_pressed_since = millis();
  } else if (_down->pushed()) {
    _down_pressed_since = millis();
  }

  switch (_currentState) {
  case AppStates::STATE_MAIN:
    processMainState();
    break;
  case AppStates::STATE_SET_TEMP:
    processSetTempState();
    break;
  case AppStates::STATE_AUTOTUNE:
    processAutoTuneState();
    break;
  case AppStates::STATE_SET_KP:
    processSetKpState();
    break;
  case AppStates::STATE_SET_KI:
    processSetKiState();
    break;
  case AppStates::STATE_SET_KD:
    processSetKdState();
    break;
  default:
    _lcd->clear();
    _currentState = AppStates::STATE_MAIN;
    break;
  }

  this->updateDisplay();
}

void StateMachine::updateDisplay() {
  writeLineToDisplay(0, "Nerdbrew");

  String boolState = "OFF";
  if (_stateValBool) {
    boolState = "ON";
  }
  String spacer = "  "; // String has some weird side-effects if you're mixing String objects and regular char*

  switch (_currentState) {
  case AppStates::STATE_MAIN:
    writeLineToDisplay(1, "Temp: " + Utils::floatToStr(_pid->getCurrentTemp()));
    writeLineToDisplay(2, "Tgt: " + Utils::floatToStr(_pid->getTargetTemp()));
    writeLineToDisplay(3, "State: " + _pid->getStateStr());
    if (_pid->getState() == PIDState::ON) {
      writeLineToDisplay(4, "Power: " + _pid->getPowerStr() + spacer);
    }
    break;
  case AppStates::STATE_SET_TEMP:
    writeLineToDisplay(1, "SET TARGET");
    writeLineToDisplay(2, Utils::floatToStr(_stateValDbl));
    break;
  case AppStates::STATE_AUTOTUNE:
    writeLineToDisplay(1, "AUTOTUNE");
    writeLineToDisplay(2, boolState);
    break;
  case AppStates::STATE_SET_KP:
    writeLineToDisplay(1, "SET KP");
    writeLineToDisplay(2, Utils::floatToStr(_stateValDbl));
    break;
  case AppStates::STATE_SET_KI:
    writeLineToDisplay(1, "SET KI");
    writeLineToDisplay(2, Utils::floatToStr(_stateValDbl));
    break;
  case AppStates::STATE_SET_KD:
    writeLineToDisplay(1, "SET KD");
    writeLineToDisplay(2, Utils::floatToStr(_stateValDbl));
    break;
  default:
    _lcd->clear();
    _currentState = AppStates::STATE_MAIN;
    break;
  }
}

void StateMachine::processMainState() {
  // If pressed the on/off button
  if (_onOffExit->pushed()) {
    clearDisplayLine(3);
    clearDisplayLine(4);
    int pidState = _pid->getState();

    // If already off, turn on. If on, turn off.
    // Ignore any other states (e.g. don't interrupt auto tuning)
    if (pidState == PIDState::OFF) {
      _pid->turnOn();
    } else if (pidState == PIDState::ON) {
      _pid->turnOff();
    }
  } else {
    // If pressed the -> button, change to next state (set temperature)
    if (_right->pushed()) {
      _lcd->clear();
      _stateValDbl = _pid->getTargetTemp();
      _currentState = AppStates::STATE_SET_TEMP;
    }
  }
}

void StateMachine::processSetTempState() {
  if (_onOffExit->pushed()) { // Exit back to main state
    _lcd->clear();
    _currentState = AppStates::STATE_MAIN;
    return;
  }

  // Jumped to the AutoTune state ?
  if (_right->pushed()) {
    _lcd->clear();
    _stateValBool = _pid->getState() == PIDState::AUTOTUNE;
    _currentState = AppStates::STATE_AUTOTUNE;
    return;
  }

  // Increasing temperature
  if (isPressed(_up, _up_pressed_since)) {
    clearDisplayLine(2);
    _stateValDbl += 1.0;
    return;
  }

  // Lowering temperature
  if (isPressed(_down, _down_pressed_since)) {
    clearDisplayLine(2);
    _stateValDbl -= 1.0;
    return;
  }

  // Set temperature
  if (_set->pushed()) {
    _lcd->clear();
    _pid->setTargetTemp(_stateValDbl);
    _currentState = AppStates::STATE_MAIN;
  }
}

void StateMachine::processAutoTuneState() {
  // If you entered autotune mode, that's it. You can't cancel
  if (_pid->getState() != PIDState::AUTOTUNE) {
    if (isPressed(_up, _up_pressed_since) || isPressed(_down, _down_pressed_since)) {
      clearDisplayLine(2);
      _stateValBool = !_stateValBool;
      return;
    }

    if (_set->pushed()) {
      _lcd->clear();
      _pid->autoTune();
      _currentState = AppStates::STATE_MAIN;
      return;
    }
  }

  if (_onOffExit->pushed()) {
    _lcd->clear();
    _currentState = AppStates::STATE_MAIN;
    return;
  }

  if (_right->pushed()) {
    _lcd->clear();
    _stateValDbl = _pid->getKp();
    _currentState = AppStates::STATE_SET_KP;
    return;
  }
}

void StateMachine::processSetKpState() {
  if (_onOffExit->pushed()) { // Exit back to main state
    _lcd->clear();
    _currentState = AppStates::STATE_MAIN;
    return;
  }

  // Jumped to the Set Ki state?
  if (_right->pushed()) {
    _lcd->clear();
    _stateValDbl = _pid->getKi();
    _currentState = AppStates::STATE_SET_KI;
    return;
  }

  // Increasing Kp
  if (isPressed(_up, _up_pressed_since)) {
    clearDisplayLine(2);
    _stateValDbl += 1.0;
    return;
  }

  // Lowering Kp
  if (isPressed(_down, _down_pressed_since)) {
    clearDisplayLine(2);
    _stateValDbl -= 1.0;
    return;
  }

  // Set Kp
  if (_set->pushed()) {
    _lcd->clear();
    _pid->setKp(_stateValDbl);
    _currentState = AppStates::STATE_MAIN;
  }
}

void StateMachine::processSetKiState() {
  if (_onOffExit->pushed()) { // Exit back to main state
    _lcd->clear();
    _currentState = AppStates::STATE_MAIN;
    return;
  }

  // Jumped to the Set Kd state?
  if (_right->pushed()) {
    _lcd->clear();
    _stateValDbl = _pid->getKd();
    _currentState = AppStates::STATE_SET_KD;
    return;
  }

  // Increasing Ki
  if (isPressed(_up, _up_pressed_since)) {
    clearDisplayLine(2);
    _stateValDbl += 0.01;
    return;
  }

  // Lowering Ki
  if (isPressed(_down, _down_pressed_since)) {
    clearDisplayLine(2);
    _stateValDbl -= 0.01;
    return;
  }

  // Set Ki
  if (_set->pushed()) {
    _lcd->clear();
    _pid->setKi(_stateValDbl);
    _currentState = AppStates::STATE_MAIN;
  }
}

void StateMachine::processSetKdState() {
  if (_onOffExit->pushed()) { // Exit back to main state
    _lcd->clear();
    _currentState = AppStates::STATE_MAIN;
    return;
  }

  // Jumped to the main state?
  if (_right->pushed()) {
    _lcd->clear();
    _currentState = AppStates::STATE_MAIN;
    return;
  }

  // Increasing Kd
  if (isPressed(_up, _up_pressed_since)) {
    clearDisplayLine(2);
    _stateValDbl += 0.01;
    return;
  }

  // Lowering Kd
  if (isPressed(_down, _down_pressed_since)) {
    clearDisplayLine(2);
    _stateValDbl -= 0.01;
    return;
  }

  // Set Kd
  if (_set->pushed()) {
    _lcd->clear();
    _pid->setKd(_stateValDbl);
    _currentState = AppStates::STATE_MAIN;
  }
}

bool StateMachine::isPressed(Button* btn, unsigned long &pressedSince) {
  bool isPressed = btn->pushed() || (btn->isPressed() && ((millis() - pressedSince) > 300));
  if (isPressed) {
    pressedSince = millis();
  }

  return isPressed;
}

void StateMachine::clearDisplayLine(int line) {
  _lcd->setCursor(0, line);
  _lcd->clearLine();
}

void StateMachine::writeLineToDisplay(int line, String value, bool clear) {
  if (clear) {
    this->clearDisplayLine(line);
  } else {
    _lcd->setCursor(0, line);
  }

  _lcd->print(value);
}
