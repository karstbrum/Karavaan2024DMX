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
void Pixels::threeColorRotation(uint8_t num_angles, float width_angle, int direction, float fadetime)
{

    // direction: clockwise of anti clockwise (1 or -1)
    // fadetime: can be any positive number which determines the time the LEDs dim to 5% of the original value
    // num_pixels: the number of pixels which should move, equidistant over the cluster range. Can be any postive integer

    // set sample time
    float Ts_ = Ts;

    // count the pulseindex normally
    // divide by number of angles to make effect not too fast
    pulseIndex += static_cast<float>(direction) * (Ts_ / 1000) * (BPM / 60) / (num_angles * 3) / freqdiv; // Ts*BPS (s^1 * s^-1)

    // if pulseindex exceeds 1, select the cluster to light up
    if (pulseIndex > 1 || pulseIndex < 0)
    {

        // depending on direction, either do + or - 1
        pulseIndex -= direction;
    }

    // determine the angles inbetween which
    float angle_start[num_angles * 3 + 1]; // +1 for ease of use later
    float angle_end[num_angles * 3];

    // define increment in angle (float)
    float angle_incr = 2.0f * PI / (num_angles * 3.0f);

    // fill in angles based on pulseIndex and number of angles
    for (uint8_t i_angle = 0; i_angle < num_angles * 2; i_angle++)
    {

        // define angle start and end
        angle_start[i_angle] = 2.0f * PI * pulseIndex + i_angle * angle_incr;
        // wrap start angle to [0, 2*pi]
        angle_start[i_angle] = angle_start[i_angle] > 2 * PI ? angle_start[i_angle] - 2 * PI : angle_start[i_angle];
        angle_start[i_angle] = angle_start[i_angle] < 0 ? angle_start[i_angle] + 2 * PI : angle_start[i_angle];

        // now add the angle width
        angle_end[i_angle] = angle_start[i_angle] + width_angle;
        // wrap end angle to [0, 2*pi]
        angle_end[i_angle] = angle_end[i_angle] > 2 * PI ? angle_end[i_angle] - 2 * PI : angle_end[i_angle];
        angle_end[i_angle] = angle_end[i_angle] < 0 ? angle_end[i_angle] + 2 * PI : angle_end[i_angle];
    }

    // define arrays
    // create vectors with 1 extra to add 2*pi
    float all_angles[num_angles * 6 + 1];
    float all_angles_aux[num_angles * 6 + 1];
    uint8_t angle_colors[num_angles * 6 + 1];
    uint8_t angle_colors_aux[num_angles * 6 + 1];
    uint8_t angle_dimmers[num_angles * 6 + 1];
    uint8_t angle_dimmers_aux[num_angles * 6 + 1];

    // make a full array with alternating angle_start and angle_end, incrementing i_angle
    // make a vector of all angles, corresponding
    for (uint8_t i_angle = 0; i_angle < num_angles * 3; i_angle++)
    {

        // fill all angles
        all_angles[i_angle * 2] = angle_start[i_angle];
        all_angles[i_angle * 2 + 1] = angle_end[i_angle];

        // also make extra vector to later find the sort index
        all_angles_aux[i_angle * 2] = angle_start[i_angle];
        all_angles_aux[i_angle * 2 + 1] = angle_end[i_angle];

        // select color based on even numbers with index/2
        // also direcly set dimmer values, based on direction
        if (i_angle % 3 == 0)
        {
            if (direction == 1)
            {
                angle_colors_aux[i_angle * 2] = 2;
                angle_colors_aux[i_angle * 2 + 1] = 2;
            }
            else
            {
                angle_colors_aux[i_angle * 2] = 0;
                angle_colors_aux[i_angle * 2 + 1] = 2;
            }
        }
        else if (i_angle % 3 == 1)
        {
            if (direction == 1)
            {
                angle_colors_aux[i_angle * 2] = 1;
                angle_colors_aux[i_angle * 2 + 1] = 1;
            }
            else
            {
                angle_colors_aux[i_angle * 2] = 2;
                angle_colors_aux[i_angle * 2 + 1] = 1;
            }
        }
        else
        {
            if (direction == 1)
            {
                angle_colors_aux[i_angle * 2] = 0;
                angle_colors_aux[i_angle * 2 + 1] = 0;
            }
            else
            {
                angle_colors_aux[i_angle * 2] = 1;
                angle_colors_aux[i_angle * 2 + 1] = 0;
            }
        }

        // fill dimmer values, independent of direction
        angle_dimmers_aux[i_angle * 2] = 0;
        angle_dimmers_aux[i_angle * 2 + 1] = 1;
    }

    // add 2*pi in the last enrty of all_angles
    all_angles[num_angles * 6] = 2.0f * PI;

    // sort the array
    int n = sizeof(all_angles) / sizeof(all_angles[0]);
    std::sort(all_angles, all_angles + n);

    // define start index
    uint8_t i_start;

    // find the start index of the sorting
    for (int i_angle = 0; i_angle < num_angles * 6; i_angle++)
    {

        if (all_angles[0] == all_angles_aux[i_angle])
        {
            i_start = i_angle;
            break;
        }
    }

    // shift all arrays by the same ammount
    for (int i_angle = 0; i_angle < num_angles * 6; i_angle++)
    {

        uint8_t i_old = i_angle + i_start;
        i_old = i_old < num_angles * 6 ? i_old : i_old - num_angles * 6;

        angle_colors[i_angle] = angle_colors_aux[i_old];
        angle_dimmers[i_angle] = angle_dimmers_aux[i_old];
    }

    // copy starting values to end, but 2*pi to angles
    angle_colors[num_angles * 6] = angle_colors[0];
    angle_dimmers[num_angles * 6] = angle_dimmers[0];

    // set the fading parameters correct
    Pixels::setAlpha(fadetime);

    // loop through all pixels
    for (uint16_t i_pixel = 0; i_pixel < totalPixels; i_pixel++)
    {

        // check conditions
        // if within angles give input of 1, otherwise 0
        // if withing even numbers, give colorindex 0
        // if within odd numbers, give coloerindex 1

        for (uint8_t i_all = 0; i_all <= num_angles * 6; i_all++)
        {

            if (pixel_pos[APOS][i_pixel] < all_angles[i_all])
            {

                // give corresponding dimmer value and color
                setDimmedRange(i_pixel, i_pixel, angle_colors[i_all], angle_dimmers[i_all]);

                break;
            }
        }
    }
}