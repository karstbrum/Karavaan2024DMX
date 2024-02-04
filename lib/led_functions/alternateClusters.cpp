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

void Pixels::alternateClusters(bool clustergroup1[MAXSIDES_L], bool clustergroup2[MAXSIDES_L], float fadetime, float on_time){

    
    // iterate on pulse index
    float Ts_ = Ts;
    pulseIndex += ((Ts_ / 1000) * (BPM / 60)) / freqdiv; // Ts*BPS (s^1 * s^-1)
    pulseIndex = pulseIndex > 1 ? pulseIndex - 1 : pulseIndex;
    
    // first check modulus condition
    bool group1_on = (on_time > 0.5 && pulseIndex <= 0.25) && pulseIndex <= 0 + (on_time/2 - 0.25) ? true : false;
    bool group2_on = (on_time > 0.5 && pulseIndex >= 0.75) && pulseIndex >= 1 - (on_time/2 - 0.25) ? true : false;

    // check default condition (extend statement with or condition)
    group1_on = group1_on || (pulseIndex >= 0.75 - on_time/2 && pulseIndex <= 0.75 + on_time/2) ? true : false;
    group2_on = group2_on || (pulseIndex >= 0.25 - on_time/2 && pulseIndex <= 0.25 + on_time/2) ? true : false;

    // count up on starting en ending pixel per side
    uint16_t pixelStart = 0;
    uint16_t pixelEnd = pixelsPerSide[0]-1;

    Pixels::setAlpha(fadetime);   

    for (uint8_t i_cluster = 0; i_cluster < numSides; i_cluster++) {
        if (clustergroup1[i_cluster] && group1_on){
            Pixels::setDimmedRange(pixelStart, pixelEnd, 0, 1);
        } else if (clustergroup2[i_cluster] && group2_on) {
            Pixels::setDimmedRange(pixelStart, pixelEnd, 1, 1);
        }
        else if (clustergroup1[i_cluster]){
            Pixels::setDimmedRange(pixelStart, pixelEnd, 0, 0);
        } 
        else if (clustergroup2[i_cluster]) {
            Pixels::setDimmedRange(pixelStart, pixelEnd, 1, 0);
        }

        // define start and end pixel of the cluster
        if (i_cluster < numSides-1){
            pixelStart += pixelsPerSide[i_cluster];
            pixelEnd = pixelStart + pixelsPerSide[i_cluster+1] -1;
        }
    }

}