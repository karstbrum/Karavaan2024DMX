#ifndef LED_FUNCTIONS_H
#define LED_FUNCTIONS_H
 
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

//#include "Adafruit_NeoPixel.h"
#include "led_driver.h"
#include <math.h>
#include <cstdlib> 
#include <ctime>
#include <random>

#define INCREMENT 0.01
#define MAXSIDES_L 50

class Pixels {

    public:
        // constructor
        Pixels(uint8_t numSides_, uint16_t pixelsPerSide_[], uint8_t numPins_, uint8_t sidesPerPin_[], uint8_t LEDPin_[], float Ts_);

        // display current color
        void activateColor();
 
        // change the color of a first index
        void changeColor(uint8_t W, uint8_t R, uint8_t G, uint8_t B);

        // dimmer values
        void setDimmer(float dimmerValue);

        // beats per minute
        void setBPM(float BPM_);

        // dimmer system
        void setDimmedRange(uint16_t index_start, uint16_t index_end, float alpha = 0, uint8_t color_index = 0,  float input = 0);

        // light functions
        // set fixed color
        void setColor(uint8_t colorIndex, float dim = 1);

        // select sideson (relative) sides to turn on, with a relative ammount of face
        void strobo(uint8_t colorIndex, uint8_t numClusters_ = 0, uint8_t clusters_[MAXSIDES_L] = {}, float fadetime = 0.1, float on_time = 1, float on_chance = 1);
        void moveClockwise(uint8_t colorIndex, uint8_t numClusters_ = 0, uint8_t clusters_[MAXSIDES_L] = {}, float fadetime = 0.1);
        void movingPixel(uint8_t colorIndex, uint8_t numClusters_, uint8_t clusters_[], uint8_t direction = 1, float fadetime = 0, uint8_t num_pixels = 1);
        
        // variables
        int clusterIndex = 0;
        // variables for pulse and updown
        float pulseIndex = 0;

        // frequency divider
        float freqdiv = 1;

        // number of sides
        uint8_t numSides;

    private:

        // basic functions
        void defineFirstColors();

        // full class variables
        uint16_t pixelsPerSide[MAXSIDES];
        uint16_t totalPixels;
        float BPM = 100;
        float Ts;

        // dimstates for slow fading
        float dimstate[MAXNUMPIXELS];

        // number of clusters to turn on - used for strobo.cpp
        uint8_t number_on = 0;
        uint8_t clusterindices[MAXSIDES_L] = {};

        // RGBW class (self made)
        RGBW* strip;

        // get random number
        float randomFloat();

};

#endif