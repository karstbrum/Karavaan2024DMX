#include <Arduino.h>

// own libraries
#include "led_functions.h"

// other libraries
#include "esp_dmx.h"
#include <WiFi.h>
#include <esp_now.h>

// Sampling time (Ts)
#define Ts 33

// send time interval
#define Tsend 250

// max numbers for settings
#define MAXCOLORS 10
#define MAXMODES 16 // to be defined

// setup DMX port and pins
const dmx_port_t dmx_num = DMX_NUM_2;
// DMX start address
const int dmx_start_addr = 0;
// number of states for LED and discoball
const int led_dmx_size = 32; // muliples of 8
const int disco_dmx_size = led_dmx_size;
// DMX size (number of addresses)
const int dmx_size = led_dmx_size + disco_dmx_size;
// DMX pins (UART)
const int tx_pin = 17;
const int rx_pin = 16;
const int rts_pin = 21;

// communication to discoball
// address to send data to:
uint8_t disco_address[] = {0x0C, 0xB8, 0x15, 0x85, 0xE4, 0xE4};

// led states (dmx)
uint8_t LEDstates[dmx_size];
// discoball states (dmx)
uint8_t discostates[dmx_size];
// define dmx received data
uint8_t dmx_data[dmx_size + 1];

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

// set up the different cores
TaskHandle_t ControllerTask;
TaskHandle_t LEDTask;

uint16_t LEDsPerSide[] = {18, 18, 18, 18, 18, 18, 18, 18, 18, 18,
                          18, 18, 18, 18, 18, 18, 18, 18, 18, 18,
                          12, 13, 17, 17, 12, 12, 13, 13, 17,
                          12, 13, 13, 12, 12, 13, 13, 13, 13, 12};
uint8_t numSides = sizeof(LEDsPerSide) / sizeof(uint16_t);
uint8_t sidesPerPin[] = {10, 10, 9, 10};
uint8_t LEDPins[] = {25, 26, 32, 33};
uint8_t numPins = sizeof(LEDPins);
Pixels LED(numSides, LEDsPerSide, numPins, sidesPerPin, LEDPins, Ts);

// pointers to objects
Pixels *LED_pointer = &LED;

// sync states with DMX controller if input changed
void sync_states()
{

  // write DMX data to LED states
  for (int i = 0; i < dmx_size / 16; i++)
  {

    for (int j = 0; j < 8; j++)
    {
      // write first 8 states to LEDstates
      LEDstates[i * 8 + j] = dmx_data[i * 16 + j + 1];
      // write next 8 states to discostates
      discostates[i * 8 + j] = dmx_data[i * 16 + j + 8 + 1];
    }
  }
}

// select the correct mode and values
// first active mode is selected by default
void select_mode()
{

  // bool for finding a non-zero state
  bool state_found = false;

  // loop through sets of 8 and check for non zero entry
  for (int i = 0; i < led_dmx_size / 8; i++)
  {

    // loop through state of the mode
    for (int j = 0; j < 8; j++)
    {

      if (LEDstates[i * 8 + j] > 0)
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
      active_states[j] = LEDstates[active_states[MODE] * 8 + j];
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
    case 0: // 
      // use clusters of a pole of a full letter
      uint8_t clusters[] = {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 4, 5, 4, 6};
      uint8_t num_clusters = sizeof(clusters)/sizeof(uint8_t);
      float fade_time = 0.05;
      float on_time = 1 - static_cast<float>(active_states[EXTRA1])/255;
      float on_chance = 1 - static_cast<float>(active_states[EXTRA2])/255;
      LED.strobo(0, num_clusters, clusters, fade_time, on_time, on_chance);
      break;
    }
}

// Task for handling the LEDs on core 1
void LightsTaskcode(void *pvParameters)
{

  // TIME VARIABLES
  // Time spent in the main loop
  int loopTime = 0;

  // another option to have a timed loop is to use vTaskDelayUntil(), have to look into it first
  for (;;)
  {

    if (millis() - loopTime >= Ts)
    {

      // reset loop time
      loopTime = millis();

      // select the correct mode
      select_mode();

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
void ControllerTaskcode(void *pvParameters)
{

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_now_init();

  // setup receiver info to channel 0
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, disco_address, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  // First, use the default DMX configuration...
  dmx_config_t config = DMX_CONFIG_DEFAULT;
  // set start address to 0
  config.dmx_start_address = dmx_start_addr;

  // ...install the DMX driver...
  dmx_driver_install(dmx_num, &config, DMX_INTR_FLAGS_DEFAULT);

  dmx_set_pin(dmx_num, tx_pin, rx_pin, rts_pin);

  // send time 
  int sendTime = 0;

  // loop and read for DMX packets
  for (;;)
  {

    dmx_packet_t packet;

    if (dmx_receive(dmx_num, &packet, DMX_TIMEOUT_TICK))
    {

      // Check that no errors occurred.
      if (packet.err == DMX_OK)
      {
        // dmx_read(dmx_num, data, packet.size);
        dmx_read_offset(dmx_num, dmx_start_addr, dmx_data, dmx_size);
        //Serial.println("data received");
      }
      else
      {
        // do something?
      }
    }
    
    // sync DMX states to
    sync_states();

    // set the constraints, such as minimum BPM
    set_constraint();
    
    // task delay for stability
    vTaskDelay(1);
  }
}

void setup()
{

  Serial.begin(115200);

  // create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
      ControllerTaskcode, /* Task function. */
      "ControllerTask",   /* name of task. */
      20000,              /* Stack size of task */
      NULL,               /* parameter of the task */
      1,                  /* priority of the task */
      &ControllerTask,    /* Task handle to keep track of created task */
      0);                 /* pin task to core 0 */
  delay(500);

  // create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
      LightsTaskcode, /* Task function. */
      "LightsTask",   /* name of task. */
      20000,          /* Stack size of task */
      NULL,           /* parameter of the task */
      1,              /* priority of the task */
      &LEDTask,       /* Task handle to keep track of created task */
      1);             /* pin task to core 1 */
  delay(500);
}

// loop can be left empty, tasks are used
void loop()
{
}