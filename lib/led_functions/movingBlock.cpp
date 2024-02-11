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
void Pixels::movingBlock(float block_size, float fadetime, float move_width, float y_range[]) {

    // direction: clockwise of anti clockwise (1 or -1)
    // fadetime: can be any positive number which determines the time the LEDs dim to 5% of the original value
    // num_pixels: the number of pixels which should move, equidistant over the cluster range. Can be any postive integer

    // set sample time
    float Ts_ = Ts;

    // count the pulseindex normally
    pulseIndex += (Ts_ / 1000) * (BPM / 60) / freqdiv; // Ts*BPS (s^1 * s^-1)

    // if pulseindex exceeds 1, select the cluster to light up
    if (pulseIndex > 1) {

        // depending on direction, either do + or - 1
        pulseIndex -= 1;

        // define a start and end position
        // get a random number between 0 and 100 and map it to [-0.4 0.4]
        uint8_t randnum = rand() % 100;
        yb_start = mapValue(0, 99, y_range[0], y_range[1], randnum);
        // define a new random number for y_end
        randnum = rand() % 100;
        yb_end = mapValue(0, 99, y_range[0], y_range[1], randnum);
        
        // select random move direction
        move_direction = (rand() % 2)*2 - 1;
    }

    // define current positions
    float x = static_cast<float>(move_direction) * (-move_width/2 + pulseIndex*move_width);
    float y = yb_start + (yb_end - yb_start)*pulseIndex;

    // define range to check
    float x_min = x-block_size/2;
    float x_max = x+block_size/2;
    float y_min = y-block_size/2;
    float y_max = y+block_size/2;

    // printf("xmin: %.3f, xmax: %.3f, ymin: %.3f, ymas: %.3f\n", x_min, x_max, y_min, y_max);

    // set the fading parameters correct
    Pixels::setAlpha(fadetime);

    // // loop through all pixels 
    for (uint16_t i_pixel = 0; i_pixel < totalPixels; i_pixel++) {

        // check conditions
        // only check if the block is inside the square
        // if inside, input = 1, else input = 0

        if (pixel_pos[XPOS][i_pixel] > x_min && pixel_pos[XPOS][i_pixel] < x_max && 
            pixel_pos[YPOS][i_pixel] > y_min && pixel_pos[YPOS][i_pixel] < y_max){
                
            //pixel on
            setDimmedRange(i_pixel, i_pixel, 0, 1);

        } else {

            //pixel off
            setDimmedRange(i_pixel, i_pixel, 0, 0);

        }

    }
    
}