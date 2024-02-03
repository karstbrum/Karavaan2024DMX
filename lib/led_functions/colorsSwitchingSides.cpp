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
void Pixels::colorsSwitchingSides(uint8_t colorIndex, uint8_t numClusters, uint8_t clusters[], uint8_t cluster_order[]) {
    uint8_t waitTime = 8; //beats
    boolean startCount = false;
    boolean phase2 = false;
    uint8_t on = 1; //beats
    uint8_t off = 0; //beats

    // define clusters
    // numClusters_ defines the number of clusters defi ned in clusters_
    // clusters_ contains the number of consecuteve sides for the cluster
    uint8_t pixelsPerCluster[MAXSIDES_L];
    uint8_t sideIndex = 0;
    for (uint8_t k = 0; k < numClusters; k++) {
        pixelsPerCluster[k] = 0;
        for (uint8_t l = 0; l < clusters[k]; l++) {
            pixelsPerCluster[k] += pixelsPerSide[sideIndex];
            sideIndex += 1;
        }
    }

    pulseIndex += (Ts / 1000) * (BPM / 60);

    // if pulseindex exceeds 1, move one cluster up
    if (pulseIndex > 1) {
        pulseIndex -= 1;
        clusterIndex += 1;
    }

    if (startCount) {
        if (pulseIndex > waitTime) {
        pulseIndex -= waitTime;
        phase2 = true;
        startCount = false;
        }
    }

    uint16_t pixelStartLeft = 0;
    uint16_t pixelEndLeft = pixelsPerCluster[0] -1;
    uint16_t pixelStartRight = totalPixels - pixelsPerCluster[numClusters -1];
    uint16_t pixelEndRight = totalPixels;

    // set all strips to off before making pattern
    strip->setColorsAll(0, 0);

    for (uint8_t i_cluster = 0; i_cluster < numClusters; i_cluster++) {
        if (clusterIndex == i_cluster) {
            strip->setRange(pixelStartLeft, pixelEndLeft, 0, on);
            strip->setRange(pixelStartRight, pixelEndRight, 1, on);

            // count to next number of pixels
            if (i_cluster < numClusters-1){
                pixelStartLeft += pixelsPerCluster[i_cluster];
                pixelEndLeft = pixelStartLeft + pixelsPerCluster[i_cluster+1] -1;
                pixelEndRight = pixelStartRight - 1;
                pixelStartRight -= pixelsPerCluster[numClusters - i_cluster];
            }
        }
    }

    // initiate wait time
    startCount = true;

    if (phase2) {
            pixelStartLeft = 0;
            pixelEndLeft = pixelsPerCluster[0] -1;
            pixelStartRight = totalPixels - pixelsPerCluster[numClusters -1];
            pixelEndRight = totalPixels;

        for (uint8_t i_cluster = 0; i_cluster < numClusters; i_cluster++) {
        if (clusterIndex == i_cluster) {
            if (i_cluster < numClusters / 2) {
                strip->setRange(pixelStartLeft, pixelEndLeft, 1, off);
                strip->setRange(pixelStartRight, pixelEndRight, 0, off);
            }
             else {
                strip->setRange(pixelStartLeft, pixelEndLeft, 1, on);
                strip->setRange(pixelStartRight, pixelEndRight, 0, on);
            }

            // count to next number of pixels
            if (i_cluster < numClusters-1){
                pixelStartLeft += pixelsPerCluster[i_cluster];
                pixelEndLeft = pixelStartLeft + pixelsPerCluster[i_cluster+1] -1;
                pixelEndRight = pixelStartRight - 1;
                pixelStartRight -= pixelsPerCluster[numClusters - i_cluster];
            }
        }
    } 
    phase2 = false;
    }

}
