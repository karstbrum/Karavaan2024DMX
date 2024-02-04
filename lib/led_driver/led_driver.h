#ifndef LED_DRIVER_H
#define LED_DRIVER_H
 
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

//#include "Adafruit_NeoPixel.h"
#include "esp32_digital_led_lib.h"
#include <math.h>

#define MAXNUMCOLORS 100
#define MAXNUMPIXELS 1000
#define MAXSIDES 100
#define MAXPINS 8

#define RISETIME 1
#define FALLTIME 0.1
 
class RGBW {

    public:
        // constructor
        RGBW(uint16_t LEDsPerPin_[], uint8_t LEDpins_[], uint8_t numPins_ = 1);

        // define the colors
        void addColor(uint8_t W, uint8_t R, uint8_t G, uint8_t B);
        void changeAddedColor(uint8_t W, uint8_t R, uint8_t G, uint8_t B, uint8_t colorIndex);

        // setting the leds
        void setStrip();

        // update colors
        void setColorsAll(uint8_t color = 0, float extraDim = 1);

        // set single pixel
        void setColorsIndividual(uint16_t k, float white, float red, float green, float blue, float extraDimmer = 1);
        void setColorsIndividualFixed(uint16_t k, uint8_t color = 0, float extraDim = 1);

        // set range of lights 
        // set start and end
        void setRange(uint16_t startLED = 0, uint16_t endLED = 1, uint8_t color = 1, float extraDim = 1);
        // center oriented
        void setRangeCenter(uint16_t center = 1, uint16_t tail = 2, uint8_t color = 1, bool fade = 0);
        // create color fading range
        void setRangeColorFade(uint16_t startLED = 0, uint16_t endLED = 1, uint8_t startColor = 0, uint8_t endColor = 1, float extraDim = 1);

        // colors
        uint16_t RGBWStates[MAXNUMPIXELS][4];

        float dimmer, prevDimmer;

        uint8_t standardColor = 0;  //index of current standard color
        uint16_t numLEDs, Ts;
        uint8_t numOfColors = 0;

        uint8_t LEDPins[MAXSIDES];
        uint16_t LEDsPerPin[MAXSIDES];
        uint8_t numPins;

    private:

        // define the strands
        strand_t STRANDS[MAXPINS];
        strand_t * strands[MAXPINS];

        //color vectors {{  W   R   G   B  }}
        float colors[MAXNUMCOLORS][4];

        uint32_t colorCode[MAXNUMPIXELS];

};




#endif