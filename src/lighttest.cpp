#include <Arduino.h>

// own libraries
#include "led_functions.h"

// 288 lights divided in 8 segments
uint16_t LEDsPerSide[] = {300}; 
uint8_t numSides = sizeof(LEDsPerSide)/sizeof(uint16_t);
uint8_t sidesPerPin[] = {1};
uint8_t LEDPins[] = {25};
uint8_t numPins = sizeof(LEDPins);
Pixels LED(numSides, LEDsPerSide, numPins, sidesPerPin, LEDPins, 1);

void setup() {

  Serial.begin(115200);

}

// loop through all colors
void loop() { 
  //white
  LED.changeColor(255, 0, 0, 0);
  LED.setColor(0, 1);
  LED.activateColor();
  sleep(5);

  //red
  LED.changeColor(0, 255, 0, 0);
  LED.setColor(0, 1);
  LED.activateColor();
  sleep(5);

  //green
  LED.changeColor(0, 0, 255, 0);
  LED.setColor(0, 1);
  LED.activateColor();
  sleep(5);

  //blue
  LED.changeColor(0, 0, 0, 255);
  LED.setColor(0, 1);
  LED.activateColor();
  sleep(5);

}