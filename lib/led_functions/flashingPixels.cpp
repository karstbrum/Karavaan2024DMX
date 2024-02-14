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

// standard color with flashing pixels
void Pixels::flashingPixels(uint8_t colorIndex, uint8_t flash_chance, float fadetime, uint8_t num_colors){

    float Ts_ = Ts;
    pulseIndex += 2 * ((Ts_ / 1000) * (BPM / 60)) / freqdiv; // Ts*BPS (s^1 * s^-1)
    
    if (pulseIndex > 1) {

        pulseIndex -= 1;

        for (uint16_t i_led = 0; i_led < totalPixels; i_led++) {
            // chance to turn on
            flash_on[i_led] = (rand() % 100) < flash_chance;
            
            // select a random color between 0 and num_colors-1
            // only to this when the light turns on
            // if the light stays off, the color remains the same as before
            if (flash_on[i_led]){
                pixelcolor[i_led] = (rand() % num_colors);
            }
            
        }

    }

    // define dimvalue upfront
    float dimvalue = 0;

    Pixels::setAlpha(fadetime);

    for (uint16_t i_led = 0; i_led < totalPixels; i_led++) {

        if (pulseIndex < 0.5  && flash_on[i_led]){
            dimvalue = 1;
        } else {
            dimvalue = 0;
        }

        // set the value of the LED
        //strip->setRange(pixelStart+i_led, pixelStart+i_led, 0, dimvalue);
        Pixels::setDimmedRange(i_led, i_led, pixelcolor[i_led], dimvalue);        

    }

}