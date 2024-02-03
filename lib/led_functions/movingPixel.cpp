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
void Pixels::movingPixel(uint8_t colorIndex, uint8_t numClusters_, uint8_t clusters_[], uint8_t direction, float fadetime, uint8_t num_pixels, bool is_disco) {

    // direction: clockwise of anti clockwise (1 or -1)
    // fadetime: can be any positive number which determines the time the LEDs dim to 5% of the original value
    // num_pixels: the number of pixels which should move, equidistant over the cluster range. Can be any postive integer

    // set sample time
    float Ts_ = Ts;

    // define clusters
    // numClusters_ defines the number of clusters defined in clusters_
    // clusters_ contains the number of consecuteve sides for the cluster
    uint8_t numClusters = numClusters_;
    uint16_t pixelsPerCluster[MAXSIDES_L];
    uint8_t sideIndex = 0;
    for (uint8_t k = 0; k < numClusters; k++) {
        pixelsPerCluster[k] = 0;
        for (uint8_t l = 0; l < clusters_[k]; l++) {
            pixelsPerCluster[k] += pixelsPerSide[sideIndex];
            sideIndex += 1;
        }
    }

    // count the pulseindex normally
    pulseIndex +=  static_cast<float>(direction) * (Ts_ / 1000) * (BPM / 60) / freqdiv; // Ts*BPS (s^1 * s^-1)

    // if pulseindex exceeds 1, select the cluster to light up
    if (pulseIndex > 1 || pulseIndex < 0) {

        // depending on direction, either do + or - 1
        pulseIndex -= direction;

    }

    // array which contains the relative position of the pixels [0 , 1]
    float pos_array[num_pixels]; 

    // distance between pixels based on the number
    float pixel_distance = 1/static_cast<float>(num_pixels);

    // fill in the pos_array 
    for (uint8_t i_pixel = 0; i_pixel < num_pixels; i_pixel++){
        
        // define the relative positions
        pos_array[i_pixel] = pulseIndex + i_pixel*pixel_distance;

        // make sure the relative position is between 0 and 1
        pos_array[i_pixel] = (pos_array[i_pixel] >= 1) ? pos_array[i_pixel] - 1 : pos_array[i_pixel];

    }

    // define first start and end pixels
    uint16_t pixelStart = 0;
    uint16_t pixelEnd = pixelsPerCluster[0] - 1;

    Pixels::setAlpha(fadetime);

    // loop through all clusters to set the moving LEDs per cluster
    for (uint8_t i_cluster = 0; i_cluster < numClusters; i_cluster++) {

        // determine the normalized distance between 2  for the cluster
        float norm_dist = 3 / static_cast<float>(pixelEnd - pixelStart);

        // Let the start be smooth by making a half sine (0 to pi give result 0 to 1)
        // loop through all LEDs in cluster to check the value
        for (uint16_t i_led = 0; i_led <= pixelEnd - pixelStart; i_led++) {

            // define normalized position of led
            float norm_pos = i_led / static_cast<float>(pixelEnd-pixelStart);

            // define dimvalue as 0, will change below if in on range
            float dimvalue = 0;

            // loop
            for (uint8_t i_pixel = 0; i_pixel < num_pixels; i_pixel++){
                
                // check if the LED is is in the (-2, +2) range of a led that should be on
                // if the led is in the range, find the correctvalue
                if (norm_pos >= (pos_array[i_pixel] - norm_dist) && norm_pos <= (pos_array[i_pixel] + norm_dist)){

                    // just make it 1 for continuity 
                    dimvalue = 1;

                }

            }

            // set the value of the LED
            //strip->setRange(pixelStart+i_led, pixelStart+i_led, 0, dimvalue);
            Pixels::setDimmedRange(pixelStart+i_led, pixelStart+i_led, 0, dimvalue);        

        }

        // define start and end pixel of the cluster
        if (i_cluster < numClusters-1){
            pixelStart += pixelsPerCluster[i_cluster];
            pixelEnd = pixelStart + pixelsPerCluster[i_cluster+1] -1;
        }

    }
    
}