#ifndef LED_AUTO_MODE_H
#define LED_AUTO_MODE_H
 
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
        Pixels(uint8_t numSides_, uint8_t pixelsPerSide_[], uint8_t numPins_, uint8_t sidesPerPin_[], uint8_t LEDPin_[], uint16_t Ts_);

        // display current color
        void activateColor();
 
        // set color
        void setColor(uint8_t colorIndex);

        // dimmer values
        void setDimmer(float dimmerValue);
        void dimUp(float increment = INCREMENT);
        void dimDown(float increment = INCREMENT);

        // beats per minute
        void setBPM(float BPM_);

        // pulsating modes
        void pulseSameColor(uint8_t colorIndex, bool fade = 0, float onValue = 0.4);
        void pulseFadeColor(uint8_t color1, uint8_t color2, bool fade = 0, float onValue = 0.4, uint8_t numClusters_ = 0, uint8_t clusters_[MAXSIDES_L] = {});
        void pulseToOtherColor(bool random = 0, bool fade = 0, float onValue = 0.4);

        // travelling sides
        void travelSides(uint8_t colorIndex, bool fade = 0, float onValue = 0.6, int direction = 1, uint8_t numClusters_ = 0, uint8_t clusters_[MAXSIDES_L] = {}, 
            bool overlapColor = 0, bool randomCluster = 0);

        // some pixels up and down
        void upDown(float tailLength, uint8_t colorIndex, bool fastUpDown = 0, bool fastOnlyUpDown = 0, uint8_t numClusters_ = 0, uint8_t clusters_[MAXSIDES_L] = {},
            bool setDirection = 0, int direction_[MAXSIDES_L] = {}, bool inverse = 0, bool setPhase = 0, float phase[MAXSIDES_L] = {}, int inverseDirection = 1);

        // full wave through letters
        void travelingWave(uint8_t colorIndex, float numOfSines = 4, int direction = 1, float offset = 0);

        // flashing pixels
        void flashingPixels(uint8_t mainColor, uint8_t flashColor, uint8_t flashChance);

        // both up and down at the same time
        void fillBoth(uint8_t colorIndex, float startPos = 0.5, float lightSize = 0.2, uint8_t flashChance = 100, bool randPos = 0, 
            uint8_t numClusters_ = 0, uint8_t clusters_[MAXSIDES_L] = {}, bool inverse = 0);

        // fill up and empty
        void fillUp(uint8_t colorIndex, bool fastUpDown = 0, bool fastOnlyUpDown = 0, 
            uint8_t numClusters_ = 0, uint8_t clusters_[MAXSIDES_L] = {}, bool setDirection = 0, int direction_[MAXSIDES_L] = {}, bool setPhase = 0, float phase[MAXSIDES_L] = {},
            int inverseDirection = 1);

        // variables
        int sideIndex = 0;
        // variables for pulse and updown
        float pulseIndex = 0;

        // frequency divider
        float freqdiv = 1;

    private:

        // basic functions
        void defineFirstColors();

        // full class variables
        uint8_t numSides, pixelsPerSide[MAXSIDES];
        uint16_t Ts, totalPixels;
        float BPM = 100;

        uint8_t pulseColorIndex = 0;

        // RGBW class (self made)
        RGBW* strip;

        // get random number
        float randomFloat();

};

#endif