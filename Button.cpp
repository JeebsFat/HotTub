#include "Button.h"

Button::Button(int ipin, int idebounce, int ibeepPin) {
  pin = ipin;
  state = LOW;
  lastState = LOW;
  debounce = idebounce;
  beepPin = ibeepPin;
  
}

void Button::init() {
  pinMode(pin, INPUT_PULLDOWN);
}

bool Button::check() {
  bool result = false;
  state = digitalRead(pin);
  if (state != lastState) {
    if (state == HIGH) {
      result = true;
      beep();
    }
    delay(debounce);
  }
  lastState = state;
  return result;
}

void Button::beep() {
    tone(beepPin, 4000, 100);
    noTone(beepPin);
}