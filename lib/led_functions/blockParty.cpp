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
#include <unordered_map>

// clusters turn on in a clockwise direction
void Pixels::blockParty(uint8_t numClusters, uint8_t clusters[], uint8_t cluster_order[], float fadetime) {
    // define clusters

    uint8_t pixelsPerCluster[MAXSIDES_L];
    uint8_t sideIndex = 0;
    for (uint8_t k = 0; k < numClusters; k++) {
        pixelsPerCluster[k] = 0;
        for (uint8_t l = 0; l < clusters[k]; l++) {
            pixelsPerCluster[k] += pixelsPerSide[sideIndex];
            sideIndex += 1;
        }
    }


    pulseIndex += ((Ts / 1000) * (BPM / 60) / freqdiv) * 2;

    if (pulseIndex > 1) {
        pulseIndex -= 1;
        clusterIndex < numClusters - 1 ? clusterIndex += 1 : clusterIndex = 0;
    }

    // turn on lights per cluster
    // define the first cluster
    uint16_t pixelStart = 0;
    uint16_t pixelEnd = pixelsPerCluster[0] -1;
    uint8_t colorIndex = clusterIndex % 2;

    setAlpha(fadetime);

    for (uint8_t i_cluster = 0; i_cluster < numClusters; i_cluster++) {
        if (clusterIndex == cluster_order[i_cluster]) {
            setDimmedRange(pixelStart, pixelEnd, 0, 1);

        } else {
            setDimmedRange(pixelStart, pixelEnd, 0, 0);

        }

        // count to next number of pixels
        if (i_cluster < numClusters-1){
            pixelStart += pixelsPerCluster[i_cluster];
            pixelEnd = pixelStart + pixelsPerCluster[i_cluster+1] -1;
        }

    }

}