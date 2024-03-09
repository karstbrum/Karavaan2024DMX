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
void Pixels::movingPixel(uint8_t colorIndex, uint8_t numClusters_, uint8_t clusters_[], int direction, float fadetime, uint8_t num_pixels, float pixelwidth)
{

    // direction: clockwise of anti clockwise (1 or -1)
    // fadetime: can be any positive number which determines the time the LEDs dim to 5% of the original value
    // num_pixels: the number of pixels which should move, equidistant over the cluster range. Can be any postive integer
    // pixelwidth: widht of single moving pixel block, does not have to be integer

    // set sample time
    float Ts_ = Ts;

    // define clusters
    // numClusters_ defines the number of clusters defined in clusters_
    // clusters_ contains the number of consecuteve sides for the cluster
    uint8_t numClusters = numClusters_;
    uint16_t pixelsPerCluster[MAXSIDES_L];
    uint8_t sideIndex = 0;
    for (uint8_t k = 0; k < numClusters; k++)
    {
        pixelsPerCluster[k] = 0;
        for (uint8_t l = 0; l < clusters_[k]; l++)
        {
            pixelsPerCluster[k] += pixelsPerSide[sideIndex];
            sideIndex += 1;
        }
    }

    // count the pulseindex normally
    pulseIndex += static_cast<float>(direction) * (Ts_ / 1000) * (BPM / 60) / static_cast<float>(num_pixels) / freqdiv; // Ts*BPS (s^1 * s^-1)

    // if pulseindex exceeds 1, select the cluster to light up
    if (pulseIndex > 1 || pulseIndex < 0)
    {

        // depending on direction, either do + or - 1
        pulseIndex -= direction;
    }

    // distance between pixels based on the number
    float pixel_distance = 1.0f / static_cast<float>(num_pixels);

    // fill in the pos_array
    for (uint8_t i_pixel = 0; i_pixel < num_pixels; i_pixel++)
    {

        // copy old value to previous array
        pos_array_prev[i_pixel] = pos_array[i_pixel];

        // define the relative positions
        pos_array[i_pixel] = pulseIndex + i_pixel * pixel_distance;

        // make sure the relative position is between 0 and 1
        pos_array[i_pixel] = (pos_array[i_pixel] > 1) ? pos_array[i_pixel] - 1 : pos_array[i_pixel];
    }

    // define first start and end pixels
    uint16_t pixelStart = 0;
    uint16_t pixelEnd = pixelsPerCluster[0] - 1;

    // configure the dim system
    Pixels::setAlpha(fadetime);

    // loop through all clusters to set the moving LEDs per cluster
    for (uint8_t i_cluster = 0; i_cluster < numClusters; i_cluster++)
    {

        // determine the normalized distance between 2  for the cluster
        float norm_dist = pixelwidth / 2 / static_cast<float>(pixelsPerCluster[i_cluster]);

        // Let the start be smooth by making a half sine (0 to pi give result 0 to 1)
        // loop through all LEDs in cluster to check the value
        for (uint16_t i_led = pixelStart; i_led <= pixelEnd; i_led++)
        {

            // define normalized position of led
            float norm_pos = (i_led - pixelStart) / static_cast<float>(pixelsPerCluster[i_cluster]);

            // define dimvalue as 0, will change below if in on range
            float dimvalue = 0;

            // loop
            for (uint8_t i_pixel = 0; i_pixel < num_pixels; i_pixel++)
            {

                // define min and max pos
                float pos_min = pos_array[i_pixel] - norm_dist;
                float pos_max = pos_array[i_pixel] + norm_dist;
                float pos_min_prev = pos_array_prev[i_pixel] - norm_dist;
                float pos_max_prev = pos_array_prev[i_pixel] + norm_dist;

                // define the range that should light up
                // the minimum (or maximum) value is determined by checking the
                // minimum value of the new pos_array an maximum of the previous
                // check if the max or min position has to jump back to 0 or 1
                if (direction == 1)
                {

                    // do some magic on the crossover from 1 to 0
                    if (pos_max < pos_max_prev)
                    {
                        if (norm_pos > pos_max_prev - 1 && norm_pos < pos_min || norm_pos > pos_max_prev - 1 && norm_pos < pos_min)
                        {
                            dimvalue = 1;
                            break;
                        }
                    }
                    else if (pos_min > pos_max_prev)
                    {
                        pos_min = pos_max_prev;
                    }
                }
                // for direction = -1, different logic is needed
                else
                {

                    // do some magic on the crossover from 1 to 0
                    if (pos_min > pos_min_prev)
                    {
                        if (norm_pos < pos_min_prev + 1 && norm_pos > pos_max || norm_pos > pos_max - 1 && norm_pos < pos_min_prev)
                        {
                            dimvalue = 1;
                            break;
                        }
                    }
                    else if (pos_max < pos_min_prev)
                    {
                        pos_max = pos_min_prev;
                    }
                }

                // check if the LED is is in the (-2, +2) range of a led that should be on
                // if the led is in the range, find the correctvalue
                if ((norm_pos > pos_min && norm_pos <= pos_max) || norm_pos < pos_max - 1 || norm_pos > pos_min + 1)
                {

                    // just make it 1 for continuity
                    dimvalue = 1;
                    break;
                }
            }

            // if (i_led == 100){
            //     printf("%.2f, ", dimvalue);
            // }
            // if (i_led == 101){
            //     printf("%.2f, ", dimvalue);
            // }
            // if (i_led == 102){
            //     printf("%.2f\n", dimvalue);
            // }

            // set the value of the LED
            // strip->setRange(pixelStart+i_led, pixelStart+i_led, 0, dimvalue);
            Pixels::setDimmedRange(i_led, i_led, 0, dimvalue);
        }

        // define start and end pixel of the cluster
        if (i_cluster < numClusters - 1)
        {
            pixelStart += pixelsPerCluster[i_cluster];
            pixelEnd = pixelStart + pixelsPerCluster[i_cluster + 1] - 1;
        }
    }
}