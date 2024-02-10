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

        // function for defining the positoins of individual LEDs
        void definePositions(float x_start[], float y_start[], float x_end[], float y_end[]);

        // display current color
        void activateColor();

        // reset pixels
        void resetPixels();
 
        // change the color of a first index
        void changeColor(uint8_t W, uint8_t R, uint8_t G, uint8_t B);

        // dimmer values
        void setDimmer(float dimmerValue);

        // beats per minute
        void setBPM(float BPM_);

        // dimmer system
        void setAlpha(float dim_time = 0);
        void setDimmedRange(uint16_t index_start, uint16_t index_end, uint8_t color_index = 0,  float input = 0);

        // light functions
        // set fixed color
        void setColor(uint8_t colorIndex, float dim = 1);

        // functions based on individual leds or clusters
        void strobo(uint8_t colorIndex, uint8_t numClusters_ = 0, uint8_t clusters_[MAXSIDES_L] = {}, float fadetime = 0.1, float on_time = 1, float on_chance = 1);
        void moveClockwise(uint8_t colorIndex, uint8_t numClusters_ = 0, uint8_t clusters_[MAXSIDES_L] = {}, uint8_t cluster_order_[MAXSIDES_L] = {}, int direction = 1, float fadetime = 0.1);
        void movingPixel(uint8_t colorIndex, uint8_t numClusters_, uint8_t clusters_[], uint8_t direction = 1, float fadetime = 0, uint8_t num_pixels = 1, bool is_disco = false);
        void flashingPixels(uint8_t colorIndex, uint8_t flash_chance, float fadetime = 0);
        void blockParty(uint8_t colorIndex, uint8_t numClusters_ = 0, uint8_t clusters_[MAXSIDES_L] = {}, uint8_t cluster_order_[MAXSIDES_L] = {}, float fadetime = 0.1);
        void alternateClusters(bool clustergroup1[MAXSIDES_L], bool clustergroup2[MAXSIDES_L], float fadetime = 0, float on_time = 0.5);

        // functions based on coordinates
        void oneColorRotation(uint8_t colorIndex, uint8_t num_angles, float width_angle, uint8_t direction, float fadetime = 0);
        void twoColorRotation(uint8_t colorIndex, uint8_t num_angles, float width_angle, uint8_t direction, float fadetime = 0);
        void movingCircles(uint8_t colorIndex, uint8_t num_circles, float circle_width, uint8_t direction, float fadetime = 0);

        // variables for counting up clusters
        int clusterIndex = 0;

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

        // pixel positions
        // x, y, l, a
        const uint8_t XPOS = 0;
        const uint8_t YPOS = 1;
        const uint8_t LPOS = 2;
        const uint8_t APOS = 3;
        float pixel_pos[4][MAXNUMPIXELS];

        // set the alpha value for dimmer
        float alpha_disc = 0;

        // dimstates for slow fading
        float dimstate[MAXNUMPIXELS];

        // variables for pulse and updown
        float pulseIndex = 0;
        float prev_pulseIndex = 0;
        float extra_pulseIndex = 0;
        bool normal_pulseIndex = true;

        // on_time / flash_time, used for flashingPixels.cpp
        bool flash_on[MAXNUMPIXELS] = {};

        // number of clusters to turn on - used for strobo.cpp
        uint8_t number_on = 0;
        uint8_t clusterindices[MAXSIDES_L] = {};

        // RGBW class (self made)
        RGBW* strip;

        // get random number
        float randomFloat();

};

// define a map function
float mapValue(float old_min, float old_max, float new_min, float new_max, float value);

#endif