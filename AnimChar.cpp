#include "AnimChar.h"

AnimChar::AnimChar(byte icustomChar[8][8], int itotalFrames) {
    customChar[8][8] = icustomChar[8][8];
    frame = 0;   
    totalFrames = itotalFrames;
}

void AnimChar::init() {
    frameTimer.start(frameTime);
}

void AnimChar::advanceFrame() {
    if (frameTimer.available()) {
        if (frame < totalFrames)    { frame = frame ++; }
        else                        { frame = 0; }
    }
}

byte AnimChar::getCurrentChar() {
    return customChar[frame][8];
}
