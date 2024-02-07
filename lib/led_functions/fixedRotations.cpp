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
void Pixels::fixedRotation(uint8_t colorIndex, uint8_t num_angles, float width_angle, uint8_t direction, float fadetime) {

    // direction: clockwise of anti clockwise (1 or -1)
    // fadetime: can be any positive number which determines the time the LEDs dim to 5% of the original value
    // num_pixels: the number of pixels which should move, equidistant over the cluster range. Can be any postive integer

    // set sample time
    float Ts_ = Ts;

    // count the pulseindex normally
    pulseIndex +=  static_cast<float>(direction) * (Ts_ / 1000) * (BPM / 60) / freqdiv; // Ts*BPS (s^1 * s^-1)

    // if pulseindex exceeds 1, select the cluster to light up
    if (pulseIndex > 1 || pulseIndex < 0) {

        // depending on direction, either do + or - 1
        pulseIndex -= direction;

    }

    // determine the angles inbetween which 
    float angle_start[num_angles*2+1];
    float angle_end[num_angles*2];

    // define increment in angle (float)
    float angle_incr = 2.0f*PI/(num_angles*2.0f);

    // fill in angles based on pulseIndex and number of angles
    for (uint8_t i_angle = 0; i_angle < num_angles*2; i_angle++){
        
        // define angle start and end
        angle_start[i_angle] = 2.0f*PI*pulseIndex + i_angle*angle_incr;
        // wrap start angle to [0, 2*pi]
        angle_start[i_angle] = angle_start[i_angle] > 2*PI ? angle_start[i_angle] - 2*PI : angle_start[i_angle];
        angle_start[i_angle] = angle_start[i_angle] < 0 ? angle_start[i_angle] + 2*PI : angle_start[i_angle];

        // now add the angle width
        angle_end[i_angle] = angle_start[i_angle] + width_angle;

    }

    // copy the first index of angle_start to the last, easy for use later
    angle_start[num_angles*2] = angle_start[0];

    // set the fading parameters correct
    Pixels::setAlpha(fadetime);

    // loop through all pixels 
    for (uint16_t i_pixel = 0; i_pixel < totalPixels; i_pixel++) {

        // check conditions
        // if within angles give input of 1, otherwise 0
        // if withing even numbers, give colorindex 0
        // if within odd numbers, give coloerindex 1

        // define a boolean to break the loop when the pixel is found
        bool pixel_found = false;

        // loop through odd numbers
        for (uint8_t i_angle = 0; i_angle < num_angles*2-1; i_angle+=2){
            
            // if between angle_start and angle_end, or smaller than angle_end-2*pi, set color 1 on 1
            if ((pixel_pos[APOS][i_pixel] > angle_start[i_angle] && pixel_pos[APOS][i_pixel] <= angle_end[i_angle]) ||
                 pixel_pos[APOS][i_pixel] <= angle_end[i_angle] - 2*PI){
                    // set the color and break for loop
                    setDimmedRange(i_pixel, i_pixel, 0, 1);
                    pixel_found = true;
                    break;

            }
            // check if between 
            else if ((pixel_pos[APOS][i_pixel] > angle_end[i_angle] && pixel_pos[APOS][i_pixel] <= angle_start[i_angle+1]) ||
                      (angle_start[i_angle + 1] < angle_end[i_angle] && 
                       (pixel_pos[APOS][i_pixel] > angle_end[i_angle] || pixel_pos[APOS][i_pixel] <= angle_start[i_angle + 1]))) {
                        // set the to zero and break the loop
                        // determine which color this should be based on the direction
                        if (direction == 1){
                            setDimmedRange(i_pixel, i_pixel, 0, 0);
                        } else {
                            setDimmedRange(i_pixel, i_pixel, 1, 0);
                        }
                        pixel_found = true;
                        break; 
            }
        }

        // if pixel is found in the odd numbers, continue to next pixel
        if (pixel_found){
            continue;
        }

        // loop through even numbers
        for (uint8_t i_angle = 1; i_angle < num_angles*2; i_angle+=2){

            // if between angle_start and angle_end, or smaller than angle_end-2*pi, set color 1 on 1
            if ((pixel_pos[APOS][i_pixel] > angle_start[i_angle] && pixel_pos[APOS][i_pixel] <= angle_end[i_angle]) ||
                 pixel_pos[APOS][i_pixel] <= angle_end[i_angle] - 2*PI){
                    // set the color and break for loop
                    setDimmedRange(i_pixel, i_pixel, 1, 1);
                    break;

            }
            // check if between 
            else if ((pixel_pos[APOS][i_pixel] > angle_end[i_angle] && pixel_pos[APOS][i_pixel] <= angle_start[i_angle+1]) ||
                      (angle_start[i_angle + 1] < angle_end[i_angle] && 
                       (pixel_pos[APOS][i_pixel] > angle_end[i_angle] || pixel_pos[APOS][i_pixel] <= angle_start[i_angle + 1]))) {
                        // set the to zero and break the loop
                        // determine which color this should be based on the direction
                        if (direction == 1){
                            setDimmedRange(i_pixel, i_pixel, 1, 0);
                        } else {
                            setDimmedRange(i_pixel, i_pixel, 0, 0);
                        }
                        break; 
            }

        }


    }
    
}