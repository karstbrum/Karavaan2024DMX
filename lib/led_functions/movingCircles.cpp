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
void Pixels::movingCircles(uint8_t num_circles, float circle_width, int direction, float fadetime, float clip_radius)
{

    // direction: clockwise of anti clockwise (1 or -1)
    // fadetime: can be any positive number which determines the time the LEDs dim to 5% of the original value
    // num_pixels: the number of pixels which should move, equidistant over the cluster range. Can be any postive integer

    // set sample time
    float Ts_ = Ts;

    // count the pulseindex normally
    pulseIndex += static_cast<float>(direction) * (Ts_ / 1000) * (BPM / 60) / static_cast<float>(num_circles) / freqdiv; // Ts*BPS (s^1 * s^-1)

    // if pulseindex exceeds 1, select the cluster to light up
    if (pulseIndex > 1 || pulseIndex < 0)
    {

        // depending on direction, either do + or - 1
        pulseIndex -= direction;
    }

    // determine the minimum and maximum radius of the circle
    float min_radius = 0 - circle_width;
    // pythagoras sqrt(r^2+r^2), r = 1+circle_width
    // this is the length towards the normalized corner
    float max_radius = std::pow(clip_radius + circle_width, 0.5);

    // define total width
    float total_width = max_radius - min_radius;

    // determine the angles inbetween which
    float circle_start[num_circles]; // +1 for ease of use later
    float circle_end[num_circles];

    // define increment in angle (float)
    float circle_incr = total_width / static_cast<float>(num_circles);

    // fill in circles based on pulseIndex and number of circles
    for (uint8_t i_circle = 0; i_circle < num_circles; i_circle++)
    {

        // define circle start and end
        circle_start[i_circle] = min_radius + total_width * pulseIndex + i_circle * circle_incr;
        // wrap start circle to [min_radius, max_radius], needed if num_cirecles > 0
        circle_start[i_circle] = circle_start[i_circle] > max_radius ? circle_start[i_circle] - total_width : circle_start[i_circle];
        circle_start[i_circle] = circle_start[i_circle] < min_radius ? circle_start[i_circle] + total_width : circle_start[i_circle];

        // now add the circle width, do not fit to range
        circle_end[i_circle] = circle_start[i_circle] + circle_width;

        // printf("start%i: %.2f, end%i: %.2f, ", i_circle, circle_start[i_circle], i_circle, circle_end[i_circle]);
    }
    // printf("\n");

    // set the fading parameters correct
    Pixels::setAlpha(fadetime);

    // // loop through all pixels
    for (uint16_t i_pixel = 0; i_pixel < totalPixels; i_pixel++)
    {

        // check conditions
        // if within circle start and end, give input 1

        // define variable for circle found
        bool circle_found = false;

        for (uint8_t i_all = 0; i_all < num_circles; i_all++)
        {

            // Check if pixel is in a circle
            if (pixel_pos[LPOS][i_pixel] >= circle_start[i_all] && pixel_pos[LPOS][i_pixel] < circle_end[i_all])
            {

                // give corresponding dimmer value and color
                setDimmedRange(i_pixel, i_pixel, 0, 1);

                // found the pixel
                circle_found = true;

                // break the circle for loop, increases speed
                break;
            }
        }

        // if pixel is not in the range of a circle, set input to zero
        if (!circle_found)
        {
            setDimmedRange(i_pixel, i_pixel, 0, 0);
        }
    }
}