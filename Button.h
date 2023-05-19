#ifndef Button_h
#define Button_h

#include <Arduino.h>
#include "MyTimers.h"

class Button {
  public:
    int state;
    int lastState;
    int pin;
    int debounce;
    int beepPin;
    Button(int ipin, int idebounce, int beepPin);
    void init();
    bool check();
    void beep();
};

#endif
