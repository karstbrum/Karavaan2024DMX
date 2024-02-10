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

// a show of blocks of 2 colors
void Pixels::blockParty(uint8_t numClusters, uint8_t clusters[], uint8_t cluster_order[], float fadetime) {

    uint8_t pixelsPerCluster[MAXSIDES_L];
    uint8_t sideIndex = 0;
    for (uint8_t k = 0; k < numClusters; k++) {
        pixelsPerCluster[k] = 0;
        for (uint8_t l = 0; l < clusters[k]; l++) {
            pixelsPerCluster[k] += pixelsPerSide[sideIndex];
            sideIndex += 1;
        }
    }

    pulseIndex += (Ts / 1000) * (BPM / 60) / 4;

    // set all strips to off before making pattern
    strip->setColorsAll(0, 0);

    uint8_t percentage = 10;
    uint8_t blocks = std::ceil(100/ percentage);
    uint16_t pixelStart = 0;
    uint16_t pixelEnd = ceil(pixelsPerCluster[0] * percentage);

    // every pulseIndex it should switch the starting color to differentiate every 4 beats
    uint8_t sequence = 0;
    setAlpha(fadetime);

    if (pulseIndex > 1) {
        pulseIndex -= 1;

        for (uint8_t i_cluster = 0; i_cluster < numClusters; i_cluster++) {
            uint16_t pixelsPerBlock = ceil(pixelsPerCluster[i_cluster] * percentage);
            for (uint8_t block = 0; block < blocks; block++) {
                if (block % 2 == i_cluster % 2) {
                    // even blocks versus even clusters, should switch the order of 2 colors per cluster
                    // color index 0
                    setDimmedRange(pixelStart, pixelEnd, sequence % 2, 1);
                } else {
                    // color index 1
                    setDimmedRange(pixelStart, pixelEnd, sequence + 1 % 2, 1);
                }
                pixelStart += pixelEnd + 1;
                pixelEnd = pixelStart + pixelsPerBlock -1;
            }
        }
        sequence += 1;
    }



}