#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "led_auto_mode.h"
#include <cmath>
#include <cstdlib> 
#include <ctime>
#include <random>


Pixels::Pixels(uint8_t numSides_, uint16_t pixelsPerSide_[], uint8_t numPins_, uint8_t sidesPerPin_[], uint8_t LEDPin_[], float Ts_) {

    Ts = Ts_; // sampling time

    // number of pixels per group which are defined as a group
    numSides = numSides_;

    // pixels per pin are needed to setup the strip
    uint16_t pixelsPerPin[numPins_] = {0};

    // total amount of pixels
    totalPixels = 0;

    for (uint16_t k = 0; k < numSides_; k++) {
        pixelsPerSide[k] = pixelsPerSide_[k];
        totalPixels += pixelsPerSide[k];
    }

    // define number of pixels per pin
    uint8_t sideIndex = 0;
    for (uint8_t pinIndex = 0; pinIndex < numPins_; pinIndex++) {
        for (uint8_t k = 0; k < sidesPerPin_[pinIndex]; k++) {
            pixelsPerPin[pinIndex] += pixelsPerSide[sideIndex];
            sideIndex += 1;
        }
    }

    strip = new RGBW(pixelsPerPin, LEDPin_, numPins_);

    Pixels::defineFirstColors();    

    // set random seed
    srand(static_cast <unsigned> (time(0)));
};

void Pixels::defineFirstColors() { 
    strip->addColor(0,   0,   0,   0); //warm white 0
};

void Pixels::changeColor(uint8_t W, uint8_t R, uint8_t G, uint8_t B){
    //Serial.printf("W: %i, R: %i, G: %i, B: %i\n", W, R, G, B);
    strip->changeAddedColor(W, R, G, B, 0);
};

void Pixels::setBPM(float BPM_) {
    BPM = BPM_;
};

void Pixels::setDimmer(float dimmerValue) {
    strip->prevDimmer = strip->dimmer;
    strip->dimmer = dimmerValue;
};

// set color
void Pixels::setColor(uint8_t colorIndex, float dim) {
    strip->setColorsAll(colorIndex, dim);
};


// pulsating color same color
void Pixels::pulseSameColor(uint8_t colorIndex, bool fade, float onValue) {
    // make a sine wave, maybe with extended max time
    
    float Ts_ = Ts;
    pulseIndex += (Ts_ / 1000) * (BPM / 60) / freqdiv; // Ts*BPS (s^1 * s^-1)
    pulseIndex = (pulseIndex > 1) ? pulseIndex -= 1 : pulseIndex;

    float dimValue = 0.525 - 0.475 * cos(pulseIndex * 2 * PI);

    if (fade == 0) {
        dimValue = (pulseIndex > onValue) ? 1 : 0;
    }
    
    strip->setColorsAll(colorIndex, dimValue);

}

// move small row up and down per side
void Pixels::upDown(float tailLength, uint8_t colorIndex, bool fastUpDown, bool fastOnlyUpDown, uint8_t numClusters_, uint8_t clusters_[],
    bool setDirection, int direction_[], bool inverse , bool setPhase, float phase_[], int inverseDirection) {

    // define clusters
    // numClusters_ defines the number of clusters defined in clusters_
    // clusters_ contains the number of consecuteve sides for the cluster
    uint8_t numClusters;
    uint16_t pixelsPerCluster[MAXSIDES_L];
    if (numClusters_ == 0) {
        numClusters = numSides;
        for (uint8_t k = 0; k < numClusters; k++) {
            pixelsPerCluster[k] = pixelsPerSide[k];
        }
    }
    else {
        numClusters = numClusters_;
        uint8_t sideIndex = 0;
        for (uint8_t k = 0; k < numClusters; k++) {
            pixelsPerCluster[k] = 0;
            for (uint8_t l = 0; l < clusters_[k]; l++) {
                pixelsPerCluster[k] += pixelsPerSide[sideIndex];
                sideIndex += 1;
            }
        }
    }

    float Ts_ = Ts;

    if (inverseDirection == -1 || inverseDirection == 0) {
        pulseIndex -= ((Ts_ / 1000) * (BPM / 60)) / 2 / freqdiv; // Ts*BPS (s^1 * s^-1)
        pulseIndex = (pulseIndex < 0) ? pulseIndex += 1 : pulseIndex;
    }
    else {
        pulseIndex += ((Ts_ / 1000) * (BPM / 60)) / 2 / freqdiv; // Ts*BPS (s^1 * s^-1)
        pulseIndex = (pulseIndex > 1) ? pulseIndex -= 1 : pulseIndex;
    }
    
    
    if (inverse) {
        strip->setColorsAll(colorIndex, 1);
    }
    else {
        strip->setColorsAll(colorIndex, 0);
    }

    //set the correct and direction (direction for fastup etc.)
    float phase[numClusters];
    int direction[numClusters];

    for (uint8_t k = 0; k < numClusters; k++) {
        if (setDirection) {
            direction[k] = direction_[k];
        }
        else {
            direction[k] = 0;
        }
        if (setPhase) {
            phase[k] = direction[k] * phase_[k];
        }
        else {
            phase[k] = 0;
        }
    }

    float pulseIndexCluster;

    for (uint8_t k = 0; k < numClusters; k++) {

        if (direction[k] == 1 || direction[k] == 0) {
            pulseIndexCluster = pulseIndex;
        }
        else {
            pulseIndexCluster = 1 - pulseIndex;
        }

        float travelRange = pixelsPerCluster[k];
        uint16_t centerFloat;

        if (fastUpDown) {
            
            float sinInput = (pulseIndexCluster + phase[k]) / 2;
            sinInput = (sinInput > 0.5) ? 0 : sinInput;
            sinInput = (sinInput < 0) ? sinInput + 0.5 : sinInput;

            float sinOutput;
            if (direction[k] == 1) {
                sinOutput = sin(sinInput * 2 * PI);
            }
            else {
                sinOutput = 1 - sin(sinInput * 2 * PI);
            }

            centerFloat = static_cast<uint16_t>(travelRange * (sinOutput));
            

        }
        else if (fastOnlyUpDown) {

            float sinInput = (pulseIndexCluster + phase[k]) / 4;
            sinInput = (sinInput > 0.25) ? 0 : sinInput;
            sinInput = (sinInput < 0) ? sinInput + 0.25 : sinInput;

            float sinOutput;
            if (direction[k] == 1) {
                sinOutput = 1 + sin((sinInput - 0.25) * 2 * PI);
            }
            else {
                sinOutput = sin((sinInput) * 2 * PI);
            }

            centerFloat = static_cast<uint16_t>(travelRange * (sinOutput));

        }
        else {
            centerFloat = static_cast<uint16_t>(travelRange / 2 + travelRange * sin((pulseIndexCluster + phase[k]) * 2 * PI) / 2);
        }

        // endLastCluster is the end of the last Cluster
        uint16_t startCluster = 0;
        for (uint16_t l = 0; l < k; l++) {
            startCluster += pixelsPerCluster[l];
        }

        // center is the center of the traveling light
        uint16_t center = startCluster + centerFloat;

        int startLED = center - static_cast<uint16_t>(travelRange * tailLength);
        int endLED = center + static_cast<uint16_t>(travelRange * tailLength);

        startLED = (startLED < startCluster) ? startCluster : startLED;
        endLED = (endLED >= (startCluster + pixelsPerCluster[k])) ? (startCluster + pixelsPerCluster[k] - 1) : endLED;

        if (inverse) {
            strip->setRange(startLED, endLED, colorIndex, 0);
        }
        else {
            strip->setRange(startLED, endLED, colorIndex, 1);
        }

    }

}

void Pixels::fillUp(uint8_t colorIndex, bool fastUpDown, bool fastOnlyUpDown,
    uint8_t numClusters_, uint8_t clusters_[], bool setDirection, int direction_[], bool setPhase, float phase_[], int inverseDirection) {
    // define clusters
    // numClusters_ defines the number of clusters defined in clusters_
    // clusters_ contains the number of consecuteve sides for the cluster
    uint8_t numClusters;
    uint16_t pixelsPerCluster[MAXSIDES_L];
    float clusterPhase[MAXSIDES_L];

    if (numClusters_ == 0) {
        numClusters = numSides;
        for (uint8_t k = 0; k < numClusters; k++) {
            pixelsPerCluster[k] = pixelsPerSide[k];
            clusterPhase[k] = 0;
        }
    }
    else {
        numClusters = numClusters_;
        uint8_t sideIndex = 0;
        for (uint8_t k = 0; k < numClusters; k++) {
            pixelsPerCluster[k] = 0;
            for (uint8_t l = 0; l < clusters_[k]; l++) {
                pixelsPerCluster[k] += pixelsPerSide[sideIndex];
                sideIndex += 1;
            }
        }
    }

    float Ts_ = Ts;
    if (inverseDirection == -1 || inverseDirection == 0) {
        pulseIndex -= ((Ts_ / 1000) * (BPM / 60)) / 2 / freqdiv; // Ts*BPS (s^1 * s^-1)
        pulseIndex = (pulseIndex < 0) ? pulseIndex += 1 : pulseIndex;
    }
    else {
        pulseIndex += ((Ts_ / 1000) * (BPM / 60)) / 2 / freqdiv; // Ts*BPS (s^1 * s^-1)
        pulseIndex = (pulseIndex > 1) ? pulseIndex -= 1 : pulseIndex;
    }

    strip->setColorsAll(colorIndex, 0);

    //set the correct and direction (direction for fastup etc.)
    float phase[numClusters];
    int direction[numClusters];

    for (uint16_t k = 0; k < numClusters; k++) {
        if (setDirection) {
            direction[k] = direction_[k];
        }
        else {
            direction[k] = 0;
        }
        if (setPhase) {
            phase[k] = direction[k] * phase_[k];
        }
        else {
            phase[k] = 0;
        }
    }

    float pulseIndexCluster;
    uint16_t leadingPixel;

    for (uint16_t k = 0; k < numClusters; k++) {

        if (direction[k] == 1 || direction[k] == 0) {
            pulseIndexCluster = pulseIndex;
        }
        else {
            pulseIndexCluster = 1 - pulseIndex;
        }

        float travelRange = pixelsPerCluster[k];

        if (fastUpDown) {

            float sinInput = (pulseIndexCluster + phase[k]) / 2;
            sinInput = (sinInput > 0.5) ? 0 : sinInput;
            sinInput = (sinInput < 0) ? sinInput + 0.5 : sinInput;

            float sinOutput;
            if (direction[k] == 1) {
                sinOutput = sin(sinInput * 2 * PI);
            }
            else {
                sinOutput = 1 - sin(sinInput * 2 * PI);
            }

            leadingPixel = static_cast<uint16_t>(travelRange * (sinOutput));

        }
        else if (fastOnlyUpDown) {

            float sinInput = (pulseIndexCluster + phase[k]) / 4;
            sinInput = (sinInput > 0.25) ? 0 : sinInput;
            sinInput = (sinInput < 0) ? sinInput + 0.25 : sinInput;

            float sinOutput;
            if (direction[k] == 1) {
                sinOutput = 1 + sin((sinInput - 0.25) * 2 * PI);
            }
            else {
                sinOutput = sin((sinInput) * 2 * PI);
            }

            leadingPixel = static_cast<uint16_t>(travelRange * (sinOutput));

        }
        else {
            leadingPixel = static_cast<uint16_t>(travelRange / 2 + travelRange * sin((pulseIndexCluster + phase[k]) * 2 * PI) / 2);
        }

        // endLastCluster is the end of the last Cluster
        uint16_t startCluster = 0;
        for (uint16_t l = 0; l < k; l++) {
            startCluster += pixelsPerCluster[l];
        }

        // center is the center of the traveling light
        uint16_t center = startCluster + leadingPixel;

        int startLED = (direction[k] == 1) ? startCluster : startCluster + leadingPixel;
        int endLED = (direction[k] == 1) ? startCluster + leadingPixel : startCluster + pixelsPerCluster[k] - 1;

        startLED = (startLED < startCluster) ? startCluster : startLED;
        endLED = (endLED >= (startCluster + pixelsPerCluster[k])) ? (startCluster + pixelsPerCluster[k] - 1) : endLED;

        strip->setRange(startLED, endLED, colorIndex, 1);

    }

}

void Pixels::travelingWave(uint8_t colorIndex, float numOfSines, int direction, float offset) {

    float Ts_ = Ts;

    if (direction == 1) {
        pulseIndex += ((Ts_ / 1000) * (BPM / 60)) / freqdiv; // Ts*BPS (s^1 * s^-1)
        pulseIndex = (pulseIndex > 1) ? pulseIndex - 1 : pulseIndex;
    }
    else if (direction == -1) {
        pulseIndex -= ((Ts_ / 1000) * (BPM / 60)) / freqdiv; // Ts*BPS (s^1 * s^-1)
        pulseIndex = (pulseIndex < 0) ? pulseIndex + 1 : pulseIndex;
    }

    uint16_t startingPixel = 0;

    float travelRange = totalPixels;
    travelRange = numOfSines/(travelRange);

    for (uint16_t k = startingPixel; k < startingPixel + totalPixels; k++) {

        float pixelValue = (0.5 + 0.5 * sin(((k - startingPixel) * travelRange + pulseIndex)*2*PI) - offset) / (1 - offset);
        pixelValue = (pixelValue < 0) ? 0 : pixelValue;
        
        strip->setColorsIndividualFixed(k, colorIndex, pixelValue);

    }


}


void Pixels::travelSides(uint8_t colorIndex, bool fade, float onValue, int direction, uint8_t numClusters_, uint8_t clusters_[], bool overlapColor, bool randomCluster) {

    // define clusters
    // numClusters_ defines the number of clusters defined in clusters_
    // clusters_ contains the number of consecuteve sides for the cluster
    uint8_t numClusters;
    uint8_t pixelsPerCluster[MAXSIDES_L];
    if (numClusters_ == 0) {
        numClusters = numSides;
        for (uint8_t k = 0; k < numClusters; k++) {
            pixelsPerCluster[k] = pixelsPerSide[k];
        }
    }
    else {
        numClusters = numClusters_;
        uint8_t sideIndex = 0;
        for (uint8_t k = 0; k < numClusters; k++) {
            pixelsPerCluster[k] = 0;
            for (uint8_t l = 0; l < clusters_[k]; l++) {
                pixelsPerCluster[k] += pixelsPerSide[sideIndex];
                sideIndex += 1;
            }
        }
    }

    float Ts_ = Ts;

    static uint8_t randomColorIndex = 1;

    pulseIndex += (Ts_ / 1000) * (BPM / 60) / freqdiv; // Ts*BPS (s^1 * s^-1)
    if (pulseIndex > 1) {
        pulseIndex -= 1;
        if (randomCluster) {
            sideIndex = rand() % numClusters;
            randomColorIndex = rand() % (strip->numOfColors - 1);
        }
        else {
            sideIndex += direction;
        }
    }

    // do not go to zero for overlapping color
    float dimValue;
    if (overlapColor) {
        dimValue = 0.55 + 0.45 * sin((pulseIndex - 0.25) * 2 * PI);
    }
    else {
        dimValue = 0.5 + 0.5 * sin((pulseIndex - 0.25) * 2 * PI);
    }

    // set direction and random color for overlap
    if (direction == 1) {
        if (sideIndex >= numClusters) {
            sideIndex = 0;
            randomColorIndex = rand() % (strip->numOfColors - 1);
        } 
    }
    else {
        if (sideIndex < 0) {
            sideIndex = numClusters - 1;
            randomColorIndex = rand() % (strip->numOfColors - 1);
        }
    }

    // only set all colors to zero if there is no overlap required
    if (overlapColor || randomCluster) {
        colorIndex = randomColorIndex;
    }

    if (!overlapColor){
        strip->setColorsAll(colorIndex, 0);
    }
    
    uint16_t pixelStart = 0;
    for (uint8_t k = 0; k < sideIndex; k++) {
        pixelStart += pixelsPerCluster[k];
    }
    uint16_t pixelEnd = 0;
    for (uint8_t k = 0; k < sideIndex+1; k++) {
        pixelEnd += pixelsPerCluster[k];
    }
    pixelEnd -= 1;

    if (fade == 0 && overlapColor == 0) {
        dimValue = (pulseIndex > onValue) ? 1 : 0;
    }

    //strip->setColorsAll(colorIndex, 0);
    strip->setRange(pixelStart, pixelEnd, colorIndex, dimValue);

}


// standard color with flashing pixels
void Pixels::flashingPixels(uint8_t mainColor, uint8_t flashColor, uint8_t flashChance) {

    float Ts_ = Ts;
    pulseIndex += ((Ts_ / 1000) * (BPM / 60)) / freqdiv; // Ts*BPS (s^1 * s^-1)
    
    if (pulseIndex > 1) {

        pulseIndex -= 1;

        strip->setColorsAll(mainColor);

        uint16_t maxSideIndex = 0;
        for (uint8_t k = 0; k < numSides; k++) {
            maxSideIndex += pixelsPerSide[k];
        }
        maxSideIndex -= 1;

        for (int k = 0; k <= maxSideIndex; k++) {
            bool flashOn = (rand() % 100) < flashChance; 
            if (flashOn) {
                strip->setColorsIndividualFixed(k, flashColor);
            }
        }
    }

}

// opening light (start in the cent go to both sides
void Pixels::fillBoth(uint8_t colorIndex, float startPos, float lightSize, uint8_t flashChance, bool randPos, uint8_t numClusters_, uint8_t clusters_[], bool inverse) {

    // define clusters
    // numClusters_ defines the number of clusters defined in clusters_
    // clusters_ contains the number of consecuteve sides for the cluster
    uint8_t numClusters;
    uint8_t pixelsPerCluster[MAXSIDES_L];
    if (numClusters_ == 0) {
        numClusters = numSides;
        for (uint8_t k = 0; k < numClusters; k++) {
            pixelsPerCluster[k] = pixelsPerSide[k];
        }
    }
    else {
        numClusters = numClusters_;
        uint8_t sideIndex = 0;
        for (uint8_t k = 0; k < numClusters; k++) {
            pixelsPerCluster[k] = 0;
            for (uint8_t l = 0; l < clusters_[k]; l++) {
                pixelsPerCluster[k] += pixelsPerSide[sideIndex];
                sideIndex += 1;
            }
        }
    }

    float Ts_ = Ts;
    // change for different unit
    static float startPositions[MAXSIDES_L] = { };
    static bool activateCluster[MAXSIDES_L] = { };

    pulseIndex += ((Ts_ / 1000) * (BPM / 60)) / 2 / freqdiv; // Ts*BPS (s^1 * s^-1)
    //pulseIndex = pulseIndex > 1 ? pulseIndex - 1 : pulseIndex;
    
    if (pulseIndex > 1) {

        pulseIndex -= 1;

        if (randPos) {
            
            // get modulus of rand number and divide by 100 to get number between 0 and 1
            for (uint8_t k = 0; k < numClusters; k++) {
                // rand() gives errors, so use something else
                startPositions[k] = randomFloat();
            }
        }
        else {
            for (uint8_t k = 0; k < numClusters; k++) {
                startPositions[k] = startPos;
            }
        }
        
        if (flashChance >= 100) {
            for (uint8_t k = 0; k < numClusters; k++) {
                activateCluster[k] = 1;
            }
        }
        else {
            for (uint8_t k = 0; k < numClusters; k++) {
                activateCluster[k] = (rand() % 100) < flashChance; 
            }
        }
        
    }

    if (inverse) {
        strip->setColorsAll(colorIndex, 1);
    }
    else {
        strip->setColorsAll(colorIndex, 0);
    }

    for (uint16_t k = 0; k < numClusters; k++) {
        if (activateCluster[k]) {

            float travelRange = static_cast<float>(pixelsPerCluster[k]);

            float sinInput = pulseIndex;
            float sinOutput;
            // shifted sine between 0 and 1
            sinOutput = (sin(sinInput * 2 * PI - 0.5 * PI) + 1) / 2;

            uint16_t centerSingle = static_cast<uint16_t>(startPositions[k] * travelRange);

            uint16_t startCluster = 0;
            uint16_t center = 0;

            for (uint16_t l = 0; l < k; l++) {
                startCluster += pixelsPerCluster[l];
            }
            center = startCluster + centerSingle;

            int startLED = center - static_cast<uint16_t>((travelRange*lightSize / 2) * sinOutput) + 1;
            int endLED = center + static_cast<uint16_t>((travelRange*lightSize / 2) * sinOutput) - 1;

            startLED = (startLED < startCluster) ? startCluster : startLED;
            endLED = (endLED >= startCluster + pixelsPerCluster[k]) ? startCluster + pixelsPerCluster[k] - 1 : endLED;

            if (inverse) {
                strip->setRange(startLED, endLED, colorIndex, 0);
            }
            else {
                strip->setRange(startLED, endLED, colorIndex, 1);
            }


        }
                    
    }

}

void Pixels::setCluster(uint8_t colorIndex, bool use_sides[MAXSIDES_L]){

    // count up on starting en ending pixel per side
    uint16_t pixelStart = 0;
    uint16_t pixelEnd = 0;

    // set all off first
    strip->setColorsAll(colorIndex, 0);

    for (uint8_t k = 0; k < numSides; k++) {
        pixelEnd = pixelStart + pixelsPerSide[k]-1;
        if (use_sides[k]){
            strip->setRange(pixelStart, pixelEnd, colorIndex, 1);
        }
        pixelStart = pixelEnd+1;
    }

}

void Pixels::switchCluster(uint8_t color1, uint8_t color2, bool use_sides1[MAXSIDES_L], bool use_sides2[MAXSIDES_L]){

    // count up on starting en ending pixel per side
    uint16_t pixelStart = 0;
    uint16_t pixelEnd = 0;

    // iterate on pulse index
    float Ts_ = Ts;
    pulseIndex += ((Ts_ / 1000) * (BPM / 60)) / freqdiv; // Ts*BPS (s^1 * s^-1)
    pulseIndex = pulseIndex > 1 ? pulseIndex - 1 : pulseIndex;

    bool switch_sides = (pulseIndex > 0.5) ? false : true;

    // set all off first
    strip->setColorsAll(color1, 0);

    for (uint8_t k = 0; k < numSides; k++) {
        pixelEnd = pixelStart + pixelsPerSide[k]-1;
        if (use_sides1[k] && switch_sides){
            strip->setRange(pixelStart, pixelEnd, color1, 1);
        } else if (use_sides2[k] && !switch_sides) {
            strip->setRange(pixelStart, pixelEnd, color2, 1);
        }
        pixelStart = pixelEnd+1;
    }

}

void Pixels::travelAround(int direction, uint8_t numColors, uint8_t colors[], uint8_t numClusters, uint8_t clusters[], float cluster_locations[],  
            float horizontal_size, float vertical_size, bool horizontal_fade, bool vertical_fade, bool use_horizontal_pos, float horizontal_pos, float vertical_pos){
    
    // iterate on pulse index, only needed if not manual
    float Ts_ = Ts;
    pulseIndex += direction * ((Ts_ / 1000) * (BPM / 60)) / freqdiv; // Ts*BPS (s^1 * s^-1)
    pulseIndex = pulseIndex > 1 ? pulseIndex - 1 : pulseIndex;
    pulseIndex = pulseIndex < 0 ? pulseIndex + 1 : pulseIndex;

    // determine clusters
    uint8_t pixelsPerCluster[MAXSIDES_L];
    uint8_t sideIndex = 0;
    for (uint8_t k = 0; k < numClusters; k++) {
        pixelsPerCluster[k] = 0;
        for (uint8_t l = 0; l < clusters[k]; l++) {
            pixelsPerCluster[k] += pixelsPerSide[sideIndex];
            sideIndex += 1;
        }
    }

    // if using horizontal position as input, overwrite pulse index
    if (use_horizontal_pos){
        pulseIndex = horizontal_pos;
    }

    //set the correct and direction (direction for fastup etc.)
    float phase[numColors];
    for (uint8_t k = 0; k < numColors; k++) {
        phase[k] = k/static_cast<float>(numColors);
    }

    // set all off first
    strip->setColorsAll(colors[0], 0);

    for (uint16_t k = 0; k < numColors; k++) {

        float travelRange = 1;
        float centerFloat_hor;
        float centerFloat_ver = vertical_pos;

        // value between 0 and 1
        centerFloat_hor = pulseIndex + phase[k];
        centerFloat_hor = (centerFloat_hor>1) ? centerFloat_hor-1 : centerFloat_hor;

        // get start and end pixel
    	uint16_t pixelStart = 0;
        uint16_t pixelEnd = 0;

        // loop through clusters
        for (uint8_t l = 0; l < numClusters; l++){
    
            // end pixel update update
            pixelEnd = pixelStart + pixelsPerCluster[l]-1;

            // when centerfloat is low (<0) value or high (>1) shift location
            float shift = 0;
            if ((centerFloat_hor-horizontal_size/2) < 0){
                float shift = centerFloat_hor - horizontal_size/2;
                if (cluster_locations[l] > (1+shift)){
                    cluster_locations[l] = cluster_locations[l]-1;
                }

            } else if ((centerFloat_hor+horizontal_size/2) > 1){
                float shift = centerFloat_hor + horizontal_size/2 - 1;
                if (cluster_locations[l] < (shift)){
                    cluster_locations[l] = cluster_locations[l]+1;
                }
            }

            // check if cluster is in range to light up,  wrapping is not needed
            if ((cluster_locations[l] > (centerFloat_hor-horizontal_size/2)) && (cluster_locations[l] < (centerFloat_hor+horizontal_size/2))){

                // determine the value of of the center position by relative position in the window, dependend on fade
                float rel_pos_hor = (cluster_locations[l]-(centerFloat_hor-horizontal_size/2))/horizontal_size;
                float centervalue_ver = (horizontal_fade) ? 0.5-0.5*cos(rel_pos_hor*2*PI) : 1;

                // determine value of all lights in the cluster based on value of center
                // loop through pixels in cluster and determine value
                for (float m = 0; m < pixelsPerCluster[l]; m++){

                    float rel_pixel_pos = m/static_cast<float>(pixelsPerCluster[l]-1);

                    // check if pixel is in range
                    if ((rel_pixel_pos > (centerFloat_ver-vertical_size/2)) && (rel_pixel_pos < (centerFloat_ver+vertical_size/2))){

                        // determine relative position and corresponding dimmer value
                        float rel_pos_ver = (rel_pixel_pos-(centerFloat_ver-vertical_size/2))/vertical_size;
                        float pixelvalue_ver = (vertical_fade) ? 0.5-0.5*cos(rel_pos_ver*2*PI) : 1;
                        strip->setColorsIndividualFixed(pixelStart+m, colors[k], pixelvalue_ver*centervalue_ver);

                    }
                }
            }

            pixelStart = pixelEnd+1;

        }
    }
}


// set the color
void Pixels::activateColor() {
    strip->setStrip();
}

// get random number
float Pixels::randomFloat() {
    uint8_t randnum = rand() % 100;
    return 0.1 + static_cast<float>(randnum) / 125;
}
