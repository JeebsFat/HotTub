#include "MyTimers.h"

void Timers::start(uint32_t waitTime) {
	_waitTime = waitTime;
	restart();
}

void Timers::restart() {
	_destTime = millis() + _waitTime;
}

void Timers::stop() {
	_waitTime = 0;
}

bool Timers::available() {
	if (_waitTime == 0) {
		return false;
	}
	return millis() >= _destTime;
}

int Timers::getMinutesLeft() {
    int value = _destTime - millis();		// millis left
	value = value / 1000 / 60;				// minutes left
	return value;
}

int Timers::getDisplayMinutes() {
    int value = getMinutesLeft();			// minutes left
	value = value % 60;						// display minutes
	return value;
}

int Timers::getDisplayHours() {
    int value = getMinutesLeft();			// minutes left
	value = value / 60;						// display hours
	return value;
}