#ifndef Button_h
#define Button_h

#include <Arduino.h>
#include "MyTimers.h"

class AnimChar {
	public:
		int frame;
		int totalFrames;
		Timers frameTimer;
		const int frameTime = 42;						// 24 fps -> 1000ms/24fps  == ~42 ms
		byte customChar[8][8];
		AnimChar(byte icustomChar[8][8], int itotalFrames);
		void init();
		void advanceFrame();
		byte getCurrentChar();
};

#endif

//byte char[8] = {0b00000,	0b00010, 0b00101, 0b00010, 0b00000, 0b01000, 0b10100, 0b01000 };
