#ifndef EDM_STATE_MACHINE_H
#define EDM_STATE_MACHINE_H

#include <inttypes.h>
#include <PCD8544.h>
#include <button.h>
#include <pid_control.h>

enum AppStates {
  STATE_MAIN = 0,
  STATE_SET_TEMP,
  STATE_AUTOTUNE,
  STATE_SET_KP,
  STATE_SET_KI,
  STATE_SET_KD
};

class StateMachine {
protected:
  int _currentState;

  // External
  PIDControl* _pid;
  PCD8544* _lcd;

  // --- Buttons
  Button* _onOffExit;
  Button* _right;
  Button* _up;
  unsigned long _up_pressed_since;
  Button* _down;
  unsigned long _down_pressed_since;
  Button* _set;

  // State values
  double _stateValDbl;
  bool _stateValBool;
  void processMainState();
  void processSetTempState();
  void processAutoTuneState();
  void processSetKpState();
  void processSetKiState();
  void processSetKdState();
  void updateDisplay();
  void clearDisplayLine(int line);
  void writeLineToDisplay(int line, String value, bool clear = false);
  bool isPressed(Button* btn, unsigned long &pressedSince);

public:
  StateMachine(int onOffBtnPin, int rightBtnPin, int upBtnPin, int downBtnPin, int setBtnPin);
  ~StateMachine();

  // --- Needs to be called before processing!!!//
  void setPIDControl(PIDControl* pid); // ------//
  void setLCD(PCD8544* lcd);           // ------//
  // -------------------------------------------//

  void process(void);
};

#endif
