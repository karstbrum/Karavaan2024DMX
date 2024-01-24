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
void Pixels::strobo(uint8_t colorIndex, uint8_t numClusters_, uint8_t clusters_[], float fadetime, float on_time, float on_chance) {

    // ontime is the relative plateua time
    // clusterson is the relative ammount of clusters that is turned on

    // set sample time
    float Ts_ = Ts;

    // set limits on on_time and on_chance
    on_time = on_time < 0.05 ? on_time : 0.05;
    on_chance = on_chance < 0.05 ? on_chance : 0.05;

    // define clusters
    // numClusters_ defines the number of clusters defined in clusters_
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

    // if pulseindex exceeds 1, select the cluster to light up
    if (pulseIndex >= 1) {

        pulseIndex -= 1;
        // determine number of clusters to be used
        // reset seed
        srand(time(0));
        // select random numbers
        for (int i = 0; i < numClusters; i++){
            boolvector[i] = rand() % 100 < static_cast<uint8_t>(on_chance*100) ? true : false;
        }
        
    }

    // adjustable function for dimvalue
    float dimValue;

    // define when on
    // fully on in ontime section, fade in small section, otherwise off
      // on
    if (pulseIndex > 0.5 - on_time/2 && pulseIndex < 0.5 + on_time/2){
        dimValue = 1;
    } // off to on
    else if(pulseIndex > 0.5 - on_time/2 - fadetime && pulseIndex < 0.5 - on_time/2) {
        dimValue = (pulseIndex - (0.5 - on_time/2 - fadetime)) / fadetime;
    } // on to off
    else if(pulseIndex < 0.5 + on_time/2 + fadetime && pulseIndex > 0.5 + on_time/2) {
        dimValue = 1 - (pulseIndex - (0.5 + on_time/2)) / fadetime;
    }  // off
    else {
        dimValue = 0;
    }

    // set all strips to off before making pattern
    strip->setColorsAll(0, 0);
    
    uint16_t pixelStart = 0;
    uint16_t pixelEnd = 0;
    for (uint8_t k = 0; k < numClusters; k++) {
        pixelStart += pixelsPerCluster[k];
        pixelEnd = pixelStart + pixelsPerCluster[k] -1;

        if(boolvector[k]){
            strip->setRange(pixelStart, pixelEnd, colorIndex, dimValue);
        }

    }
    
}