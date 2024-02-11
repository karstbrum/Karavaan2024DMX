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

void Pixels::resetPixels(){
    strip->resetPixels();
}

void Pixels::definePositions(float x_start[], float y_start[], float x_end[], float y_end[]){

    // The first and last LED are defined on:
    // (x, y) = (x_start + x_spacing/2, y_start + y_spacing/2)
    // (x, y) = (x_end   - x_spacing/2, y_end   - y_spacing/2)

    // define pixel index
    uint16_t i_pixel = 0;

    // loop through all sides
    for (uint16_t i_side = 0; i_side < numSides; i_side++) {

        // define the spacing between the les on the current side
        float x_spacing = (x_end[i_side]-x_start[i_side]) / (pixelsPerSide[i_side]);
        float y_spacing = (y_end[i_side]-y_start[i_side]) / (pixelsPerSide[i_side]);

        float x_first = x_start[i_side] + x_spacing/2;
        float y_first = y_start[i_side] + y_spacing/2;

        // loop through all pixels on the current side
        for (uint16_t k = 0; k < pixelsPerSide[i_side]; k++){
            
            // define x and y position of the pixel
            float x = x_first + k*x_spacing;
            float y = y_first + k*y_spacing;

            // define distance to (0,0)
            // just take pythagoras of x and y position, x^2 + y^2
            float xysquared = std::pow(x, 2) + std::pow(y, 2);
            // take the root of x^2 + y^2
            float l = std::pow(xysquared, 0.5);

            // define the angle.
            // start (x,y) = (+x,0), counterclockwise
            float a = acos(x/l);

            // if the y position is positive, a is within [0    pi]
            // if the y position is negative, a is within [pi 2*pi]
            a = y < 0 ? 2*PI-a : a;

            // add numbers to pos data array
            pixel_pos[XPOS][i_pixel] = x;
            pixel_pos[YPOS][i_pixel] = y;
            pixel_pos[LPOS][i_pixel] = l;
            pixel_pos[APOS][i_pixel] = a;

            printf("x: %f, y: %f, l: %f, a: %f\n", x, y, l, a);

            // increment pixels
            i_pixel++;

        }

    }

}

void Pixels::defineFirstColors() { 
    strip->addColor(0,   0,   0,   0); 
    strip->addColor(0,   0,   0,   0); 
    strip->addColor(0,   0,   0,   0); 
};

void Pixels::changeColor(uint8_t W, uint8_t R, uint8_t G, uint8_t B){
    // Serial.printf("W: %i, R: %i, G: %i, B: %i\n", W, R, G, B);
    strip->changeAddedColor(W, R, G, B, 0);
    strip->changeAddedColor(W, B, R, G, 1);
    strip->changeAddedColor(W, G, B, R, 2);
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

void Pixels::setAlpha(float dim_time){
    // calculate the alpha discrete value based on alpha
    // alpha is the actual time it takes to go from 1 to 0.01
    // number of samples to to go to the values
    float num_samples = dim_time/Ts*1000 - 1;
    // get the alpha value, only calculate when num_samples > 0
    alpha_disc = (num_samples > 0) ? std::pow(0.001, 1/num_samples) : 0;

}

void Pixels::setDimmedRange(uint16_t index_start, uint16_t index_end, uint8_t color_index,  float input){

    for(uint16_t i_led = index_start; i_led<=index_end; i_led++){
        // this approach will add the input to the brightness, so if input is e.g. 0.5
        // you get a funky effect
        // x[k] = a*x[k-1] + u
        // dimstate has the value of previous iteration, set to new iteration
        dimstate[i_led] = alpha_disc*dimstate[i_led] + input;
        // limit the dimstate to a max of 1
        dimstate[i_led] = dimstate[i_led] > 1 ? 1 : dimstate[i_led];
        // set color and correct dimvalue to the LED
        strip->setColorsIndividualFixed(i_led, color_index, dimstate[i_led]);
    }

}



// define general function
// map to new range
float mapValue(float old_min, float old_max, float new_min, float new_max, float value) {

    // calculate relative position
    float rel_value = (value - old_min) / (old_max - old_min);

    // map to new range
    float new_value = new_min + (new_max - new_min) * rel_value;

    // return new value
    return new_value;

}