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

// One of more traveling LEDs/pixels over the whole range, or per cluster
void Pixels::updownPositionBased(float updown_time, float fadetime, float phase, float line_width, float y_range[]) {

    // phase: value between -1 and 1. 

    // set sample time
    float Ts_ = Ts;

    // count the pulseindex normally
    pulseIndex += (Ts_ / 1000) * (BPM / 60) / freqdiv; // Ts*BPS (s^1 * s^-1)

    // if pulseindex exceeds 1, select the cluster to light up
    if (pulseIndex > 1) {

        // depending on direction, either do + or - 1
        pulseIndex -= 1;

    }

    // define a start and stoptime (relative) for the up down movement
    float start_time = 0.5 - updown_time/2;
    float end_time = 0.5 + updown_time/2;

    // define y_top and y_bottom for the line
    float y_center;
    float y_top;
    float y_bottom;

    // set the fading parameters correct
    Pixels::setAlpha(fadetime);

    // // loop through all pixels 
    for (uint16_t i_pixel = 0; i_pixel < totalPixels; i_pixel++) {

        // define a relative x position between 0 and 1 (can just add 0.5)
        float rel_x = pixel_pos[XPOS][i_pixel] + 0.5;

        // mulitply the relative x position with the phase difference
        // add to the pulse index to get the new pulse index
        float pulseIndex_new = pulseIndex + rel_x*phase;

        // phase can be positive and negative, so check top and bottom
        pulseIndex_new = pulseIndex_new > 1 ? pulseIndex_new - 1 : pulseIndex_new;
        pulseIndex_new = pulseIndex_new < 0 ? pulseIndex_new + 1 : pulseIndex_new;

        // get the required y position
        // use (-cos(x)+1)/2 t0 map (0, 2*pi) to (-linewidth/2, 1 + linewidth/2)
        // only use this between
        if (pulseIndex_new >= start_time && pulseIndex_new <= end_time) {

            // map starttime to endtime to [0, 2*pi]
            float use_phase = mapValue(start_time , end_time, 0, 2*PI, pulseIndex_new);

            // make the range a little larger to make the effect smooth
            y_center = ((-cos(use_phase)+1)/2);
            
            // map the value to the y_range
            y_center = mapValue(0 , 1, y_range[0]-line_width/2, y_range[1]+line_width/2, y_center);

            // define top and bottom of the line
            y_bottom = y_center - line_width/2;
            y_top = y_center + line_width/2;

        } 
        // else just turn off
        else {
            y_bottom = -1;
            y_top = -1;
        }

        if (pixel_pos[YPOS][i_pixel] >= y_bottom && pixel_pos[YPOS][i_pixel] <= y_top){
                
            //pixel on
            setDimmedRange(i_pixel, i_pixel, 0, 1);

        } else {

            //pixel off
            setDimmedRange(i_pixel, i_pixel, 0, 0);

        }

    }
    
}
