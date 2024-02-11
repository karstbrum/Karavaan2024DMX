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
void Pixels::movingLines(uint8_t number_of_lines, uint8_t direction, float fadetime, float linewidth) {

    // direction: 1, 2, 3, or 4
    // number of lines can be anything 

    // set sample time
    float Ts_ = Ts;

    // count the pulseindex normally
    pulseIndex += (Ts_ / 1000) * (BPM / 60) / 4 / freqdiv; // Ts*BPS (s^1 * s^-1)

    // if pulseindex exceeds 1, select the cluster to light up
    if (pulseIndex > 1) {

        // depending on direction, either do + or - 1
        pulseIndex -= 1;

    }

    // define distance between lines
    float line_distance = 0.1 + linewidth;

    // the 'screen' has a 1 by 1 size
    // let the lines move over a distance of 2 instead of 1, use pulseIndex for this
    // define the distance based on line_distance and number of lines
    // give 0.5 extra distance on default
    float half_dist = 0.75 + line_distance * number_of_lines;

    // direction 1 and 2 are in positive direction
    // direction 3 and 4 are negative, map the new pulseIndex accordingly
    int direction_pn;
    float pulseIndex_new;
    if (direction == 1 || direction == 2){
        pulseIndex_new = mapValue(0, 1, -half_dist, half_dist, pulseIndex);
        direction_pn = 1;
    } else {
        pulseIndex_new = mapValue(0, 1, half_dist, -half_dist, pulseIndex);
        direction_pn = -1;
    }

    // select what to do based on direction
    // 1 and 3 are x direction
    uint8_t pixel_dir;
    if (direction == 1 || direction == 3){
        pixel_dir = XPOS;
    } else {
        pixel_dir = YPOS;
    }

    // set the fading parameters correct
    Pixels::setAlpha(fadetime);

    // // loop through all pixels 
    for (uint16_t i_pixel = 0; i_pixel < totalPixels; i_pixel++) {

        // boolean for when the pixel is on a line
        bool on_line = false;

        for (uint8_t i_line = 0; i_line < number_of_lines; i_line++){
            
            // define the negative and positive bound of the line
            float line_center = pulseIndex_new - i_line*line_distance;
            float negative = line_center - linewidth/2;
            float positive = line_center + linewidth/2;

            if (pixel_pos[pixel_dir][i_pixel] >= negative && pixel_pos[pixel_dir][i_pixel] <= positive){
                
                //pixel on
                on_line = true;
                // break the loop for speed
                break;

            } 

        }

        if (on_line){
                
            //pixel on
            setDimmedRange(i_pixel, i_pixel, 0, 1);

        } else {

            //pixel off
            setDimmedRange(i_pixel, i_pixel, 0, 0);

        }

    }
    
}
