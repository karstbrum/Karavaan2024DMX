#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "led_auto_mode.h"
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

// 
void Pixels::strobo(uint8_t colorIndex, uint8_t numClusters_, uint8_t clusters_[], float fadetime, float on_time, float on_chance) {

    // ontime is the relative plateua time
    // clusterson is the relative ammount of clusters that is turned on

    // set sample time
    float Ts_ = Ts;

    // define clusters
    // numClusters_ defines the number of clusters defined in clusters_
    // clusters_ contains the number of consecuteve sides for the cluster
    uint8_t numClusters = numClusters_;
    uint8_t pixelsPerCluster[MAXSIDES_L];
    uint8_t sideIndex = 0;
    for (uint8_t k = 0; k < numClusters; k++) {
        pixelsPerCluster[k] = 0;
        for (uint8_t l = 0; l < clusters_[k]; l++) {
            pixelsPerCluster[k] += pixelsPerSide[sideIndex];
            sideIndex += 1;
        }
    }

    pulseIndex += (Ts_ / 1000) * (BPM / 60) / freqdiv; // Ts*BPS (s^1 * s^-1)

    // if pulseindex exceeds 1, select the cluster to light up
    if (pulseIndex > 1) {

        pulseIndex -= 1;
        // determine number of clusters to be used
        // reset seed
        srand(time(0));
        // select random numbers
        for (int i = 0; i < numClusters; i++){
            boolvector[i] = rand() % 100 < static_cast<uint8_t>(on_chance*100) ? true : false;
        }
        
    }

    // adjustable function for dimvalue
    float dimValue;
    // define when on
    // fully on in ontime section, fade in small section, otherwise off
    if (pulseIndex > 0.5 - on_time/2 && pulseIndex < 0.5 + on_time/2){
        dimValue = 1;
    } else if(pulseIndex > 0.5 - on_time/2 - fadetime && pulseIndex < 0.5 - on_time/2) {
        dimValue = 0;
    }   else if(pulseIndex < 0.5 + on_time/2 + fadetime && pulseIndex > 0.5 + on_time/2) {
        dimValue = 0;
    } else {
        dimValue = 0;
    }

    // set all strips to off before making pattern
    strip->setColorsAll(0, 0);
    
    uint16_t pixelStart = 0;
    uint16_t pixelEnd = 0;
    for (uint8_t k = 0; k < numClusters; k++) {
        pixelStart += pixelsPerCluster[k];
        pixelEnd = pixelStart + pixelsPerCluster[k] -1;

        if(boolvector[k]){
            strip->setRange(pixelStart, pixelEnd, colorIndex, dimValue);
        }

    }
    
}

// set the color
void Pixels::activateColor() {
    strip->setStrip();
}

// get random number
float Pixels::randomFloat() {
    uint8_t randnum = rand() % 100;
    return 0.1 + static_cast<float>(randnum) / 125;
}
