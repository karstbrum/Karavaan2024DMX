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
void Pixels::strobo(uint8_t colorIndex, uint8_t numClusters_, uint8_t clusters_[], float ramptime, float on_time, float on_chance, float fadetime) {

    // ontime is the relative plateua time
    // clusterson is the relative ammount of clusters that is turned on

    // set sample time
    float Ts_ = Ts;

    // set limits on on_time and on_chance
    on_time = on_time < 0.05 ? 0.05 : on_time;

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

    // define number of clusters that should turn on, round to upper
    number_on = ceil(on_chance*numClusters);
    // number of cluster turning on should be at least 1 and max numClusters
    number_on = number_on > 1 ? number_on : 1;
    number_on = number_on < numClusters ? number_on : numClusters;

    // count to 1 every BPM / freqdiv
    pulseIndex += (Ts_ / 1000) * (BPM / 60) / freqdiv; // Ts*BPS (s^1 * s^-1)

    // if pulseindex exceeds 1, select the cluster to light up
    if (pulseIndex >= 1) {

        pulseIndex -= 1;
        // determine number of clusters to be used
        
        // fill index vector for all clusters
        for (uint8_t i_cluster = 0; i_cluster < numClusters; i_cluster++){
            clusterindices[i_cluster] = i_cluster;
        }
        // shuffle array 
        std::random_shuffle(&clusterindices[0], &clusterindices[numClusters]);

    }

    // adjustable function for dimvalue
    float dimValue;

    // define when on
    // fully on in ontime section, fade in small section, otherwise off
      // on
    if (pulseIndex > 0.5 - on_time/2 && pulseIndex < 0.5 + on_time/2){
        dimValue = 1;
    } // off to on
    else if(pulseIndex > 0.5 - on_time/2 - ramptime && pulseIndex < 0.5 - on_time/2) {
        dimValue = (pulseIndex - (0.5 - on_time/2 - ramptime)) / ramptime;
    } // on to off
    else if(pulseIndex < 0.5 + on_time/2 + ramptime && pulseIndex > 0.5 + on_time/2) {
        dimValue = 1 - (pulseIndex - (0.5 + on_time/2)) / ramptime;
    }  // off
    else {
        dimValue = 0;
    }

    // configure the dim system
    Pixels::setAlpha(fadetime);
    
    // define first start and end pixels
    uint16_t pixelStart = 0;
    uint16_t pixelEnd = pixelsPerCluster[0] - 1;

    for (uint8_t i_cluster = 0; i_cluster < numClusters; i_cluster++) {
        
        // check if cluster should turn on
        bool cluster_on = false;
        for (uint8_t i_rand = 0; i_rand < number_on; i_rand++){
            if (i_cluster == clusterindices[i_rand]){
                cluster_on = true;
            }
        }

        if(cluster_on){
            Pixels::setDimmedRange(pixelStart, pixelEnd, 0, dimValue);   
        } else {
            Pixels::setDimmedRange(pixelStart, pixelEnd, 0, 0); 
        }

        // define start and end pixel of the cluster
        if (i_cluster < numClusters-1){
            pixelStart += pixelsPerCluster[i_cluster];
            pixelEnd = pixelStart + pixelsPerCluster[i_cluster+1] -1;
        }

    }
    
}