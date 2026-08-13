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
void Pixels::movingLines(uint8_t number_of_lines, uint8_t direction, float fadetime, float linewidth)
{

    // direction: 1 (+x), 2 (+y), 3 (-x), 4 (-y)
    // number_of_lines can be anything

    // define min and max position based on defined max and x/y direction
    const float pos_min = y_min;
    const float pos_max = y_max;
    if (direction == 1 || direction == 3)
    {
        const float pos_min = x_min;
        const float pos_max = x_max;
    }
    const float pos_span = pos_max - pos_min;

    // split the combined direction selector into axis + sign
    uint8_t pixel_dir = (direction == 1 || direction == 3) ? XPOS : YPOS;
    int sign = (direction == 1 || direction == 2) ? 1 : -1;

    // set sample time
    float Ts_ = Ts;

    // count the pulseindex normally
    pulseIndex += static_cast<float>(sign) * (Ts_ / 1000) * (BPM / 60) / freqdiv; // Ts*BPS (s^1 * s^-1)

    // if pulseindex exceeds 1, wrap it around
    if (pulseIndex > 1 || pulseIndex < 0)
    {

        // depending on direction, either do + or - 1
        pulseIndex -= sign;
    }

    // define increment in position between lines
    float pos_incr = pos_span / number_of_lines;

    // fill in line start/end positions based on pulseIndex and number of lines
    for (uint8_t i_line = 0; i_line < number_of_lines; i_line++)
    {

        // define line start and end
        line_start[i_line] = pos_min + pos_span * pulseIndex + i_line * pos_incr;
        // now add the line width
        line_end[i_line] = line_start[i_line] + linewidth;

        // if line_start is larger than the previous line_end, clamp it there
        // this has to be done before wrapping, and is direction dependent,
        // so the line can never skip past where the previous frame ended
        // (avoids gaps at high BPM, same trick as oneColorRotation)
        if (sign == 1 && line_start[i_line] > prev_line_end[i_line])
        {
            line_start[i_line] = prev_line_end[i_line];
        }
        else if (sign == -1 && prev_line_start[i_line] > line_end[i_line])
        {
            line_end[i_line] = prev_line_start[i_line];
        }

        // wrap start position to [pos_min, pos_max]
        line_start[i_line] = line_start[i_line] > pos_max ? line_start[i_line] - pos_span : line_start[i_line];
        line_start[i_line] = line_start[i_line] < pos_min ? line_start[i_line] + pos_span : line_start[i_line];
        // wrap end position to [pos_min, pos_max]
        line_end[i_line] = line_end[i_line] > pos_max ? line_end[i_line] - pos_span : line_end[i_line];
        line_end[i_line] = line_end[i_line] < pos_min ? line_end[i_line] + pos_span : line_end[i_line];

        // copy to previous line end/start
        prev_line_end[i_line] = line_end[i_line];
        prev_line_start[i_line] = line_start[i_line];
    }

    // define arrays
    // create vectors with 1 extra to add pos_max
    float all_positions[number_of_lines * 2 + 1];
    float all_positions_aux[number_of_lines * 2 + 1];
    uint8_t position_dimmers[number_of_lines * 2 + 1];
    uint8_t position_dimmers_aux[number_of_lines * 2 + 1];

    // make a full array with alternating line_start and line_end, incrementing i_line
    for (uint8_t i_line = 0; i_line < number_of_lines; i_line++)
    {

        // fill all positions
        all_positions[i_line * 2] = line_start[i_line];
        all_positions[i_line * 2 + 1] = line_end[i_line];

        // also make extra vector to later find the sort index
        all_positions_aux[i_line * 2] = line_start[i_line];
        all_positions_aux[i_line * 2 + 1] = line_end[i_line];

        // fill dimmer values, independent of direction
        position_dimmers_aux[i_line * 2] = 0;
        position_dimmers_aux[i_line * 2 + 1] = 1;
    }

    // add pos_max in the last entry of all_positions
    all_positions[number_of_lines * 2] = pos_max;

    // sort the array
    int n = sizeof(all_positions) / sizeof(all_positions[0]);
    std::sort(all_positions, all_positions + n);

    // define start index
    uint8_t i_start = 0;

    // find the start index of the sorting
    for (int i_line = 0; i_line < number_of_lines * 2; i_line++)
    {

        if (all_positions[0] == all_positions_aux[i_line])
        {
            i_start = i_line;
            break;
        }
    }

    // shift all arrays by the same amount
    for (int i_line = 0; i_line < number_of_lines * 2; i_line++)
    {

        uint8_t i_old = i_line + i_start;
        i_old = i_old < number_of_lines * 2 ? i_old : i_old - number_of_lines * 2;

        position_dimmers[i_line] = position_dimmers_aux[i_old];
    }

    // copy starting values to end, but pos_max to positions
    position_dimmers[number_of_lines * 2] = position_dimmers[0];

    // set the fading parameters correct
    Pixels::setAlpha(fadetime);

    // loop through all pixels
    for (uint16_t i_pixel = 0; i_pixel < totalPixels; i_pixel++)
    {

        // check conditions
        // if within positions give input of 1, otherwise 0

        for (uint8_t i_all = 0; i_all <= number_of_lines * 2; i_all++)
        {

            if (pixel_pos[pixel_dir][i_pixel] < all_positions[i_all])
            {

                // give corresponding dimmer value and color
                setDimmedRange(i_pixel, i_pixel, 0, position_dimmers[i_all]);

                break;
            }
        }
    }
}
