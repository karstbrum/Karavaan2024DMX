#include <Arduino.h>

// own libraries
#include "led_functions.h"

// communication libraries
#include<WiFi.h>
#include<esp_now.h>

// Sampling time (Ts)
#define Ts 20

// discoball states 
const uint8_t disco_dmx_size = 32;
uint8_t discostates[disco_dmx_size];

// define active states (are used by the lights)
const uint8_t BPM = 0;
const uint8_t DIM = 1;
const uint8_t RED = 2;
const uint8_t GREEN = 3;
const uint8_t BLUE = 4;
const uint8_t WHITE = 5;
const uint8_t EXTRA1 = 6;
const uint8_t EXTRA2 = 7;
const uint8_t MODE = 8;
uint8_t active_states[9];

// define tasks (multicore)
TaskHandle_t LEDTask;
TaskHandle_t ControllerTask;

// 288 lights divided in 8 segments
uint16_t LEDsPerSide[] = {36, 36, 36, 36, 36, 36, 36, 36}; 
uint8_t numSides = sizeof(LEDsPerSide)/sizeof(uint16_t);
uint8_t sidesPerPin[] = {8};
uint8_t LEDPins[] = {33};
uint8_t numPins = sizeof(LEDPins);
Pixels LED(numSides, LEDsPerSide, numPins, sidesPerPin, LEDPins, Ts);

// output pin for motor
uint8_t motor_pin = 32;

// level to turn motor on
uint8_t motor_on_level = 140;

// pointers to objects
Pixels* LED_pointer = &LED;

// TIME VARIABLES
// Time spent in the main loop
int loopTime = 0;

// data receive function
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  
  // map data to discostates
  for(int i=0;i<disco_dmx_size;i++){
    discostates[i] = incomingData[i];
  }

} 

// select the correct mode and values
// first active mode is selected by default
void select_mode()
{

  // bool for finding a non-zero state
  bool state_found = false;

  // loop through sets of 8 and check for non zero entry
  for (int i = 0; i < disco_dmx_size / 8; i++)
  {
    // loop through state of the mode
    for (int j = 0; j < 8; j++)
    {
      if (discostates[i * 8 + j] > 0)
      {
        // mode found at nonzero state
        state_found = true;

        // break this loop
        break;
      }
    }
    // break loop if state is found, needed for later
    if (state_found)
    {
      // set the mode
      active_states[MODE] = i;

      break;
    }
  }

  // if state found set all states
  if (state_found)
  {
    for (int j = 0; j < 8; j++)
    {
      // set the values
      active_states[j] = discostates[active_states[MODE] * 8 + j];
    }
  }
  else
  {
    for (int j = 0; j < 8; j++)
    {
      // set the values
      active_states[j] = 0;
    }
  }
}

void set_constraint()
{
  // set a BPM of at least 1
  active_states[BPM] = active_states[BPM] > 1 ? active_states[BPM] : 1;
}

void setmode(){
  switch (active_states[MODE])
    {
    case 0: {// 
      // use clusters of a pole of a full letter
      uint8_t clusters[] = {1, 1, 1, 1, 1, 1, 1, 1};
      uint8_t num_clusters = sizeof(clusters)/sizeof(uint8_t);
      float fade_time = 0.05;
      float on_time = 1-mapValue(0, 255, 0, 1, active_states[EXTRA1]); 
      float on_chance = 1-mapValue(0, 255, 0, 1, active_states[EXTRA2]);
      LED.strobo(0, num_clusters, clusters, fade_time, on_time, on_chance);
      break; }

    case 1: {// 
      // use clusters of a pole of a full letter
      uint8_t clusters[] = {8};
      uint8_t num_clusters = sizeof(clusters)/sizeof(uint8_t);
      int direction = 1;
      // between 0 and 0.99
      float fadetime = mapValue(0, 255, 0, 0.99, active_states[EXTRA1]);
      // between 1 and 4
      uint8_t num_pixels = (uint8_t)mapValue(0, 255, 1, 4, active_states[EXTRA2]);
      LED.movingPixel(0, num_clusters, clusters, direction, fadetime, num_pixels, true);
      break; }

    case 2: {// 
      // between 0 and 0.99
      float fadetime = mapValue(0, 255, 0, 0.99, active_states[EXTRA1]);
      // flash chance between 5 and 75 %
      uint8_t flash_chance = (uint8_t)mapValue(0, 255, 5, 75, active_states[EXTRA2]);
      LED.flashingPixels(0, flash_chance, fadetime);
      break; }

    }
}

void setmotor(){
// if motor level is equal or greater than motor_on_level, turn motor on
  if(active_states[DIM] >= motor_on_level){
    digitalWrite(motor_pin, HIGH);
  } else {
    digitalWrite(motor_pin, LOW);
  }

}

// Task for handling the LEDs on core 1
void LightsTaskcode( void * pvParameters ){

  pinMode(motor_pin, OUTPUT);

  // another option to have a timed loop is to use vTaskDelayUntil(), have to look into it first
  for(;;){
    if(millis()-loopTime >= Ts){

      loopTime = millis();

      // select the correct mode
      select_mode();

      // set the motor
      setmotor();

      // set dimmer value (should be between 0 and 1)
      LED.setDimmer((static_cast<float>(active_states[DIM]))/255);

      // set color
      LED.changeColor(active_states[WHITE], active_states[RED], active_states[GREEN], active_states[BLUE]);

      // set BPM
      LED.setBPM(active_states[BPM]);

      // set mode
      setmode();

      // set the LED 
      LED.activateColor();

    }

  }
}

// handle DMX on core 0
void ControllerTaskcode( void * pvParameters ){ 

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_now_init(); 

  esp_now_register_recv_cb(OnDataRecv);

  // loop and read for DMX packets
  for(;;){
    
    // task delay for stability
    vTaskDelay(1);

  }

}


void setup() {

  Serial.begin(115200);

  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    ControllerTaskcode, /* Task function. */
                    "ControllerTask",   /* name of task. */
                    20000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &ControllerTask,    /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(500);

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    LightsTaskcode, /* Task function. */
                    "LightsTask",   /* name of task. */
                    20000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &LEDTask,     /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
  delay(500); 

  
}

// loop can be left empty, tasks are used
void loop() { 

}