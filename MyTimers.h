#ifndef MyTimers_h
#define MyTimers_h

#include <Arduino.h>
#include <inttypes.h>

class Timers
{
  public:
    Timers() {}
	
	void start(uint32_t waitTime);
	void restart();
	void stop();
	bool available();
    int getMinutesLeft();
	int getDisplayMinutes();
	int getDisplayHours();	
	
  private:
	uint32_t _waitTime;
	uint32_t _destTime;
};
#endif