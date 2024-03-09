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
void Pixels::moveClockwise(uint8_t numClusters, uint8_t clusters[], uint8_t cluster_order[], int direction, float fadetime, float cluster_length)
{
    // define clusters

    // cluster_length: relative number of clusters that are switched in order before selecting a random start position

    uint8_t pixelsPerCluster[MAXSIDES_L];
    uint8_t sideIndex = 0;
    for (uint8_t k = 0; k < numClusters; k++)
    {
        pixelsPerCluster[k] = 0;
        for (uint8_t l = 0; l < clusters[k]; l++)
        {
            pixelsPerCluster[k] += pixelsPerSide[sideIndex];
            sideIndex += 1;
        }
    }

    // finish full circle in 2 beats
    pulseIndex += ((Ts / 1000) * (BPM / 60) / freqdiv) * numClusters / 2;

    // int initialClusterIndex = direction == 1 ? clusterIndex : numClusters;
    int initialClusterIndex = 0;
    // if pulseindex exceeds 1, move one cluster
    if (pulseIndex > 1)
    {
        pulseIndex -= 1;

        if (direction == 1)
        {
            clusterIndex < numClusters - 1 ? clusterIndex += 1 : clusterIndex = 0;
        }
        else
        {
            clusterIndex > 0 ? clusterIndex -= 1 : clusterIndex = numClusters - 1;
        }

        // increment on cluster_counter
        cluster_counter += 1;

        if (cluster_length < 1 && static_cast<float>(numClusters) * cluster_length <= cluster_counter)
        {

            // reset the cluster counter
            cluster_counter = 0;

            // select a random start cluster within defined range
            clusterIndex = rand() % numClusters;
        }
    }

    // turn on lights per cluster
    // define the first cluster
    uint16_t pixelStart = 0;
    uint16_t pixelEnd = pixelsPerCluster[0] - 1;

    setAlpha(fadetime);

    for (uint8_t i_cluster = 0; i_cluster < numClusters; i_cluster++)
    {
        float onOrOff = i_cluster == cluster_order[clusterIndex] ? 1 : 0;
        setDimmedRange(pixelStart, pixelEnd, 0, onOrOff);

        // count to next number of pixels
        if (i_cluster < numClusters - 1)
        {
            pixelStart += pixelsPerCluster[i_cluster];
            pixelEnd = pixelStart + pixelsPerCluster[i_cluster + 1] - 1;
        }
    }
}