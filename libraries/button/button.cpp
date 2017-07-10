#include "button.h"
#include <inttypes.h>
#include <Arduino.h>
#include <Bounce2.h>

Button::Button(int pin) {
  _pin = pin;
  _bounce = new Bounce();
  _bounce->interval(5);
  _bounce->attach(_pin);
}

Button::~Button() {
  delete _bounce;
}

void Button::process() {
  _bounce->update();
}

// YES, this is all backwards. That's because the buttons are pulled high, and pressing them causes the pin to turn low.
bool Button::isPressed() {
  return _bounce->read() == LOW;
}

bool Button::pushed() {
  return _bounce->fell();
}

bool Button::released() {
  return _bounce->rose();
}
