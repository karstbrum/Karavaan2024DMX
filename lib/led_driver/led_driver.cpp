#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "led_driver.h"
#include <math.h>

RGBW::RGBW(uint16_t LEDsPerPin_[], uint8_t LEDpins_[], uint8_t numPins_)
{

    // get total number of LEDS
    numLEDs = 0;
    numPins = numPins_;

    Serial.begin(115200);

    // init led driver (sub library)
    digitalLeds_initDriver();

    // loop through different connected strips and setup
    for (uint8_t k = 0; k < numPins; k++)
    {

        // define number of LEDs, pins and pin numbers
        numLEDs += LEDsPerPin_[k];
        LEDsPerPin[k] = LEDsPerPin_[k];
        LEDPins[k] = LEDpins_[k];

        // add strips to the pins
        // just use k as RMT channel
        STRANDS[k] = {.rmtChannel = k, .gpioNum = LEDPins[k], .ledType = LED_SK6812W_V1, .brightLimit = 64, .numPixels = LEDsPerPin[k]};
        // set ouput to low
        gpioSetup(STRANDS[k].gpioNum, OUTPUT, LOW);
        // pointer to strands
        strands[k] = &STRANDS[k];
    }
    // add strands can use rc for troubleshooting
    int rc = digitalLeds_addStrands(strands, numPins);

    // standard dimmer off, have to set in main code
    // dimmer is the max brightness, extradimmer is meant for setting certain modes
    dimmer = 0;
    prevDimmer = 0;

    // add fully off as a color, will be overwritten by addColor()
    colors[0][0] = 0;
    colors[0][1] = 0;
    colors[0][2] = 0;
    colors[0][3] = 0;

    // RGBW::setColorsAll();
};

void RGBW::resetPixels()
{
    // reset the strands
    digitalLeds_resetPixels(strands, numPins);
}

void RGBW::addColor(uint8_t W, uint8_t R, uint8_t G, uint8_t B)
{
    colors[numOfColors][0] = W;
    colors[numOfColors][1] = R;
    colors[numOfColors][2] = G;
    colors[numOfColors][3] = B;

    numOfColors += 1;
}

void RGBW::changeAddedColor(uint8_t W, uint8_t R, uint8_t G, uint8_t B, uint8_t colorIndex)
{
    colors[colorIndex][0] = W;
    colors[colorIndex][1] = R;
    colors[colorIndex][2] = G;
    colors[colorIndex][3] = B;
}

void RGBW::setColorsAll(uint8_t color, float extraDim)
{

    for (uint16_t k = 0; k < numLEDs; k++)
    {

        RGBW::setColorsIndividual(k, colors[color][0], colors[color][1], colors[color][2], colors[color][3], extraDim);
    };
};

void RGBW::setColorsIndividualFixed(uint16_t k, uint8_t color, float extraDim)
{

    RGBW::setColorsIndividual(k, colors[color][0], colors[color][1], colors[color][2], colors[color][3], extraDim);
}

void RGBW::setColorsIndividual(uint16_t k, float white, float red, float green, float blue, float extraDimmer)
{

    RGBWStates[k][0] = static_cast<uint8_t>(dimmer * extraDimmer * white);
    RGBWStates[k][1] = static_cast<uint8_t>(dimmer * extraDimmer * red);
    RGBWStates[k][2] = static_cast<uint8_t>(dimmer * extraDimmer * green);
    RGBWStates[k][3] = static_cast<uint8_t>(dimmer * extraDimmer * blue);

    colorCode[k] = (RGBWStates[k][0] << 24) |
                   (RGBWStates[k][1] << 16) |
                   (RGBWStates[k][2] << 8) |
                   RGBWStates[k][3];
};

void RGBW::setRange(uint16_t startLED, uint16_t endLED, uint8_t color, float extraDim)
{

    for (uint16_t k = startLED; k <= endLED; k++)
    { // For each pixel in range

        RGBW::setColorsIndividual(k, colors[color][0], colors[color][1], colors[color][2], colors[color][3], extraDim);
    };
};

void RGBW::setRangeCenter(uint16_t center, uint16_t tail, uint8_t color, bool fade)
{

    float dim = 1;

    for (int k = center - tail; k <= center + tail; k++)
    {
        if (fade)
        {
            float offDim = k;
            dim = 1 / (1 + abs(offDim));
        };

        RGBW::setColorsIndividual(k, colors[color][0], colors[color][1], colors[color][2], colors[color][3], dim);
    };
};

void RGBW::setRangeColorFade(uint16_t startLED, uint16_t endLED, uint8_t startColor, uint8_t endColor, float extraDim)
{

    for (int k = startLED; k <= endLED; k++)
    {

        float totalLEDs = endLED - startLED;
        float currentLED = endLED - k;

        float WFade = colors[startColor][0] + ((colors[endColor][0] - colors[startColor][0]) * currentLED / totalLEDs);
        float RFade = colors[startColor][1] + ((colors[endColor][1] - colors[startColor][1]) * currentLED / totalLEDs);
        float GFade = colors[startColor][2] + ((colors[endColor][2] - colors[startColor][2]) * currentLED / totalLEDs);
        float BFade = colors[startColor][3] + ((colors[endColor][3] - colors[startColor][3]) * currentLED / totalLEDs);

        RGBW::setColorsIndividual(k, WFade, RFade, GFade, BFade, extraDim);
    };
};

void RGBW::setStrip()
{

    // loop through states of all pixels
    uint16_t pixelIndex = 0;

    for (uint8_t k = 0; k < numPins; k++)
    { // for each strip
        for (uint16_t l = 0; l < LEDsPerPin[k]; l++)
        { // For each pixel in strip.
            strands[k]->pixels[l].w = RGBWStates[pixelIndex][0];
            strands[k]->pixels[l].r = RGBWStates[pixelIndex][1];
            strands[k]->pixels[l].g = RGBWStates[pixelIndex][2];
            strands[k]->pixels[l].b = RGBWStates[pixelIndex][3];
            pixelIndex += 1;
            
        }
    }

    // set the colors of all strands
    digitalLeds_drawPixels(strands, numPins);

    // for simulation purpose
    // print all r g b w states '
    // printf("b");
    // for(uint16_t i_pixel = 0; i_pixel < numLEDs; i_pixel++)
    // {
    //     printf("%i,%i,%i,%i;", RGBWStates[i_pixel][0], RGBWStates[i_pixel][1], RGBWStates[i_pixel][2], RGBWStates[i_pixel][3]);
    // }
    // printf("\n");

};
