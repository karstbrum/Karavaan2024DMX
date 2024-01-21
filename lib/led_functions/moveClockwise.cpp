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

// clusters turn on in a clockwise direction
void Pixels::moveClockwise(uint8_t colorIndex, uint8_t numClusters_, uint8_t clusters_[], float fadetime) {
    // set sample time
    float Ts_ = Ts;

    // define clusters
    // numClusters_ defines the number of clusters defi ned in clusters_
    // clusters_ contains the number of consecuteve sides for the cluster
    uint8_t numClusters = numClusters_;
    uint8_t pixelsPerCluster[MAXSIDES_L];
    uint8_t sideIndex = 0;
    for (uint8_t k = 0; k < numClusters; k++) {
        pixelsPerCluster[k] = 0;
        for (uint8_t l = 0; l < clusters_[k]; l++) {
            pixelsPerCluster[k] += pixelsPerSide[sideIndex];
            sideIndex += 1;
        }
    }

    pulseIndex += (Ts_ / 1000) * (BPM / 60) / freqdiv; // Ts*BPS (s^1 * s^-1)


    // set all strips to off before making pattern
    strip->setColorsAll(0, 0);

    uint8_t currentCluster = 0;
    // if pulseindex exceeds 1, move one cluster up
    if (pulseIndex > 1) {
        pulseIndex -= 1;
        // determine number of clusters to be used
        // reset seed
        srand(time(0));
        // select random numbers
        currentCluster += 1;
    }

    // fadetime 0  0.5 1
    float getDimValue(uint8_t i) {
        // functie hier
    }

    // turn on current cluster
    uint16_t pixelStart = 0;
    uint16_t pixelEnd = 0;
    for (uint8_t k = 0; k < numClusters; k++) {
        pixelStart += pixelsPerCluster[k];
        pixelEnd = pixelStart + pixelsPerCluster[k] -1;

        if(currentCluster == k){
            strip->setRange(pixelStart, pixelEnd, colorIndex, 1);
        }

    }


    
}