#ifndef EDM_BUTTON_H
#define EDM_BUTTON_H

#include <inttypes.h>
#include <Bounce2.h>

class Button {
private:
  Bounce* _bounce;
  int _pin;

public:
  Button(int pin);
  ~Button();

  void process();
  bool isPressed();
  bool pushed();
  bool released();
};

#endif
