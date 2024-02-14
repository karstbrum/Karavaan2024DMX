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

// shows pulsating pixels based on the bpm. inverse can turn it around and show a different color
void Pixels::heartbeat(float line_size, float fadetime, bool inverse) {

    // set sample time
    float Ts_ = Ts;

    // count the pulseindex normally
    pulseIndex += (Ts_ / 1000) * (BPM / 60) / freqdiv; // Ts*BPS (s^1 * s^-1)

    // if pulseindex exceeds 1, select the cluster to light up
    if (pulseIndex > 1) {
        pulseIndex -= 1;
    }

    // define current positions of line
    float y = 0;

    // define range to check
    float y_min = y-line_size * pulseIndex;
    float y_max = y+line_size * pulseIndex;

    // set the fading parameters correct
    Pixels::setAlpha(fadetime);

    // // loop through all pixels 
    for (uint16_t i_pixel = 0; i_pixel < totalPixels; i_pixel++) {
        if (inverse) {
            if (pixel_pos[YPOS][i_pixel] > y_min && pixel_pos[YPOS][i_pixel] < y_max){
                //pixel on
                setDimmedRange(i_pixel, i_pixel, 0, 1);
            } else {
                //pixel off
                setDimmedRange(i_pixel, i_pixel, 0, 0);
            }
        } else {
            if (pixel_pos[YPOS][i_pixel] > y_min && pixel_pos[YPOS][i_pixel] < y_max){
                //pixel off
                setDimmedRange(i_pixel, i_pixel, 1, 0);
            } else {
                //pixel on
                setDimmedRange(i_pixel, i_pixel, 1, 1);
            }
        }
    }
    
}