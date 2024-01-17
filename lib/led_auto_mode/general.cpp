#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "led_functions.h"
#include <cmath>
#include <cstdlib> 
#include <ctime>
#include <random>
#include <algorithm>


Pixels::Pixels(uint8_t numSides_, uint16_t pixelsPerSide_[], uint8_t numPins_, uint8_t sidesPerPin_[], uint8_t LEDPin_[], float Ts_) {

    Ts = Ts_; // sampling time

    // number of pixels per group which are defined as a group
    numSides = numSides_;

    // pixels per pin are needed to setup the strip
    uint16_t pixelsPerPin[numPins_] = {0};

    // total amount of pixels
    totalPixels = 0;

    for (uint16_t k = 0; k < numSides_; k++) {
        pixelsPerSide[k] = pixelsPerSide_[k];
        totalPixels += pixelsPerSide[k];
    }

    // define number of pixels per pin
    uint8_t sideIndex = 0;
    for (uint8_t pinIndex = 0; pinIndex < numPins_; pinIndex++) {
        for (uint8_t k = 0; k < sidesPerPin_[pinIndex]; k++) {
            pixelsPerPin[pinIndex] += pixelsPerSide[sideIndex];
            sideIndex += 1;
        }
    }

    strip = new RGBW(pixelsPerPin, LEDPin_, numPins_);

    Pixels::defineFirstColors();    

    // set random seed
    srand(static_cast <unsigned> (time(0)));
};

void Pixels::defineFirstColors() { 
    strip->addColor(0,   0,   0,   0); //warm white 0
};

void Pixels::changeColor(uint8_t W, uint8_t R, uint8_t G, uint8_t B){
    //Serial.printf("W: %i, R: %i, G: %i, B: %i\n", W, R, G, B);
    strip->changeAddedColor(W, R, G, B, 0);
};

void Pixels::setBPM(float BPM_) {
    BPM = BPM_;
};

void Pixels::setDimmer(float dimmerValue) {
    strip->prevDimmer = strip->dimmer;
    strip->dimmer = dimmerValue;
};

// set  fixed color
void Pixels::setColor(uint8_t colorIndex, float dim) {
    strip->setColorsAll(colorIndex, dim);
};

// set the color
void Pixels::activateColor() {
    strip->setStrip();
}

// get random number
float Pixels::randomFloat() {
    uint8_t randnum = rand() % 100;
    return 0.1 + static_cast<float>(randnum) / 125;
}
