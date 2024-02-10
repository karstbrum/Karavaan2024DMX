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

void Pixels::everyXseconds(float fadetime) {
        Pixels::everyXBeats(4, [&strip, totalPixels, fadetime]() {
            strip->setRange(0, totalPixels, 0, fadetime);
});
}