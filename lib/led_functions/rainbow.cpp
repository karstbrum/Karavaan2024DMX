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

// flashing clusters, ammount can be selected
void Pixels::rainbow(float blend_level, int direction)
{

    // ontime is the relative plateua time
    // clusterson is the relative ammount of clusters that is turned on

    // set sample time
    float Ts_ = Ts;

    // count to 1 every 0.5 BPM / freqdiv
    pulseIndex += direction * (Ts_ / 1000) * (BPM / 60) / freqdiv; // Ts*BPS (s^1 * s^-1)

    // if pulseindex exceeds 1, select the cluster to light up
    if (pulseIndex > 1 || pulseIndex < 0)
    {

        pulseIndex -= direction;

    }

    // set the values for different colors
    // blend level of 0 is square, 1 is triangular, inbetween is trapezoid
    // set red (defined between 0 and 2/3)
    float red, green, blue, relative_pulse_index;

    // red
    // ramp up
    if (pulseIndex >= 0 && pulseIndex < 1.0/3.0 - 1.0/3.0*(1-blend_level))
    {
        relative_pulse_index = pulseIndex - 0.0;
        red = 0.0 + 255.0 / (1.0/3.0 * blend_level) * relative_pulse_index;
        // printf("1\n");
    }
    // constant value
    else if (pulseIndex >= 1.0/3.0 - 1.0/3.0*(1.0-blend_level) && pulseIndex < 1.0/3.0)
    {
        red = 255.0;
        // printf("2\n");
    }
    // ramp down
    else if (pulseIndex >= 1.0/3.0 && pulseIndex < 1.0/3.0 + 1.0/3.0* blend_level)
    {
        relative_pulse_index = pulseIndex - 1.0/3.0;
        red = 255.0 - 255.0 / (1.0/3.0 * blend_level) * relative_pulse_index;
        // printf("3\n");
    }
    // off
    else{
        red = 0.0;
        // printf("4\n");
    }

    // green
    // ramp up
    if (pulseIndex >= 1.0/3.0 && pulseIndex < 2.0/3.0 - 1.0/3.0*(1.0-blend_level))
    {
        relative_pulse_index = pulseIndex - 1.0/3.0;
        green = 0.0 + 255.0 / (1.0/3.0 * blend_level) * relative_pulse_index;
    }
    // constant value
    else if (pulseIndex >= 2.0/3.0 - 1.0/3.0*(1.0-blend_level) && pulseIndex < 2.0/3.0)
    {
        green = 255.0;
    }
    // ramp down
    else if (pulseIndex >= 2.0/3.0 && pulseIndex < 2.0/3.0 + 1.0/3.0* blend_level)
    {
        relative_pulse_index = pulseIndex - 2.0/3.0;
        green = 255.0 - 255.0 / (1.0/3.0 * blend_level) * relative_pulse_index;
    }
    // off
    else{
        green = 0.0;
    }

    // blue
    // ramp up
    if (pulseIndex >= 2.0/3.0 && pulseIndex < 1.0 - 1.0/3.0*(1.0-blend_level))
    {
        relative_pulse_index = pulseIndex - 2.0/3.0;
        blue = 0.0 + 255.0 / (1.0/3.0 * blend_level) * relative_pulse_index;
    }
    // constant value
    else if (pulseIndex >= 1.0 - 1.0/3.0*(1.0-blend_level) && pulseIndex <= 1.0)
    {
        blue = 255.0;
    }
    // ramp down
    else if (pulseIndex >= 0.0 && pulseIndex < 0.0 + 1.0/3.0* blend_level)
    {
        relative_pulse_index = pulseIndex - 0.0;
        blue = 255.0 - 255.0 / (1.0/3.0 * blend_level) * relative_pulse_index;
    }
    // off
    else{
        blue = 0.0;
    }

    // set the color
    // else normalize red green and blue to 255
    float max_color = red;
    max_color = green > max_color ? green : max_color;
    max_color = blue > max_color ? blue : max_color;

    // define normalization factor
    // divide all colors by max color and multiply by 255
    red = round(red / max_color * 255.0f);
    green = round(green / max_color * 255.0f);
    blue = round(blue / max_color * 255.0f);

    uint8_t red_i = static_cast<uint8_t>(red);
    uint8_t green_i = static_cast<uint8_t>(green);
    uint8_t blue_i = static_cast<uint8_t>(blue);

    // overwrite color, don't use white
    Pixels::changeColor(0, red_i, green_i, blue_i);

    Pixels::setColor(0, 1);

}