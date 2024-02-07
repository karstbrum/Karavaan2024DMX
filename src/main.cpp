#include <Arduino.h>

// own libraries
#include "led_functions.h"

// other libraries
#include "esp_dmx.h"
#include <WiFi.h>
#include <esp_now.h>


// Sampling time (Ts)
#define Ts 10

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
// address to send data to: A8:42:E3:8D:B8:04 
uint8_t disco_address[] = {0xA8, 0x42, 0xE3, 0x8D, 0xB8, 0x04};

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

// bottom post y start and end (bottom to top)
float y_b1 = -0.2;
float y_b2 = 0;

// top post y start and end (bottom to top)
float y_t1 = 0;
float y_t2 = 0.2;

// x positions start & diff
float x_1 = 0.2;
float x_d = (0.5-x_1)/4;

// bottom letter (bottom to top)
float yl_b1 = -0.3;
float yl_b2 = -0.15;

// top letter (bottom to top)
float yl_t1 = -0.15;
float yl_t2 = 0;
// for the R
float yl_t3 = (yl_t1/2) ;
float yl_t4 = (yl_t1/2);

// start position 0, only diff
float xl_d = (0.5-x_1)/4;

// define relative start and end position of the sides
float start_pos_x[] = {-x_1, -x_1,  -x_1-1*x_d, -x_1-1*x_d, -x_1-2*x_d, -x_1-2*x_d, -x_1-3*x_d, -x_1-3*x_d, -x_1-4*x_d, -x_1-4*x_d,
                        x_1,  x_1,   x_1+1*x_d,  x_1+1*x_d,  x_1+2*x_d,  x_1+2*x_d,  x_1+3*x_d,  x_1+3*x_d,  x_1+4*x_d,  x_1+4*x_d,
                       -4*xl_d,         -4*xl_d,        -3*xl_d,        -4*xl_d, -2*xl_d, -2*xl_d, -2*xl_d,         -1*xl_d, -2*xl_d,
                          xl_d,  5.0f/4.0f*xl_d, 6.0f/4.0f*xl_d, 7.0f/4.0f*xl_d,  3*xl_d,  3*xl_d,  3*xl_d,  7.0f/2.0f*xl_d,  4*xl_d, 4*xl_d};
float start_pos_y[] = {y_b1, y_t1, y_b1, y_t1, y_b1, y_t1, y_b1, y_t1, y_b1, y_t1,
                       y_b1, y_t1, y_b1, y_t1, y_b1, y_t1, y_b1, y_t1, y_b1, y_t1,
                       yl_b1, yl_t1, yl_t2, yl_b2, yl_b1, yl_t1, yl_t2, yl_t4, yl_b2,
                       yl_t2, yl_b2, yl_b1, yl_t1, yl_b1, yl_t1, yl_t2, yl_b2, yl_b1, yl_t1};
float end_pos_x[] = {-x_1, -x_1,  -x_1-1*x_d, -x_1-1*x_d, -x_1-2*x_d, -x_1-2*x_d, -x_1-3*x_d, -x_1-3*x_d, -x_1-4*x_d, -x_1-4*x_d,
                      x_1,  x_1,   x_1+1*x_d,  x_1+1*x_d,  x_1+2*x_d,  x_1+2*x_d,  x_1+3*x_d,  x_1+3*x_d,  x_1+4*x_d,  x_1+4*x_d,
                             -4*xl_d,         -4*xl_d,        -4*xl_d,  -3*xl_d, -2*xl_d, -2*xl_d,        -1*xl_d, -2*xl_d, -1*xl_d,
                      5.0f/4.0f*xl_d,  6.0f/4.0f*xl_d, 7.0f/4.0f*xl_d,   2*xl_d,  3*xl_d,  3*xl_d, 7.0f/2.0f*xl_d,  4*xl_d,  4*xl_d, 4*xl_d};
float end_pos_y[] = {y_b2, y_t2, y_b2, y_t2, y_b2, y_t2, y_b2, y_t2, y_b2, y_t2,
                     y_b2, y_t2, y_b2, y_t2, y_b2, y_t2, y_b2, y_t2, y_b2, y_t2,
                     yl_b2, yl_t2, yl_t1, yl_b1, yl_b2, yl_t2, yl_t1, yl_t3, yl_b1,
                     yl_t1, yl_b1, yl_b2, yl_t2, yl_b2, yl_t2, yl_t1, yl_b1, yl_b2, yl_t2};

// define number of leds per side
uint16_t LEDsPerSide[] = {18, 18, 18, 18, 18, 18, 18, 18, 18, 18,
                          18, 18, 18, 18, 18, 18, 18, 18, 18, 18,
                          12, 13, 17, 17, 12, 12, 13, 13, 17,
                          12, 13, 13, 12, 12, 13, 13, 13, 13, 12};
// get number of sides
uint8_t numSides = sizeof(LEDsPerSide) / sizeof(uint16_t);
uint8_t sidesPerPin[] = {10, 10, 9, 10};
uint8_t LEDPins[] = {25, 26, 32, 33};
uint8_t numPins = sizeof(LEDPins);
Pixels LED(numSides, LEDsPerSide, numPins, sidesPerPin, LEDPins, Ts);

// the LED positions are defined in the setup loop
// can only declare variables in global space

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
    case 0: {
      // use clusters of a pole of a full letter
      uint8_t clusters[] = {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 4, 5, 4, 6};
      uint8_t num_clusters = sizeof(clusters)/sizeof(uint8_t);
      float fade_time = 0.05;
      float on_time = 1-mapValue(0, 255, 0, 1, active_states[EXTRA1]);
      float on_chance = 1-mapValue(0, 255, 0, 1, active_states[EXTRA2]);
      LED.strobo(0, num_clusters, clusters, fade_time, on_time, on_chance);
      break; }

    case 1: {
      // use clusters of a pole of a full letter
      uint8_t clusters[] =      {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 4, 5, 4, 6};
      uint8_t cluster_order[] = {4, 3, 2, 1, 0, 11, 10, 12, 13, 5, 6, 7, 8, 9};
      uint8_t num_clusters = sizeof(clusters)/sizeof(uint8_t);
      float fade_time = mapValue(0, 255, 0, 5, active_states[EXTRA1]);
      LED.moveClockwise(0, num_clusters, clusters, cluster_order, fade_time);
      break; }

    case 2: {
      // between 0 and 0.9
      float fadetime = mapValue(0, 255, 0, 5, active_states[EXTRA1]);
      // flash chance between 5 and 75
      uint8_t flash_chance = (uint8_t)mapValue(0, 255, 5, 75, active_states[EXTRA2]);
      LED.flashingPixels(0, flash_chance, fadetime);
      break; }

    case 3: {
      // use clusters of a pole of a full letter
      uint8_t clusters[] =      {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 4, 5, 4, 6};
      uint8_t cluster_order[] = {4, 3, 2, 1, 0, 11, 10, 12, 13, 5, 6, 7, 8, 9};
      uint8_t num_clusters = sizeof(clusters)/sizeof(uint8_t);
      float fadetime = 1;
      LED.blockParty(0, num_clusters, clusters, cluster_order, fadetime);
      break; }

    }
}

// Task for handling the LEDs on core 1
void LightsTaskcode(void *pvParameters)
{

  // define LED positions
  LED.definePositions(start_pos_x, start_pos_y, end_pos_x, end_pos_y);

  // TIME VARIABLES
  // Time spent in the main loop
  int loopTime = 0;

  // another option to have a timed loop is to use vTaskDelayUntil(), have to look into it first
  for (;;)
  {

    if (millis() - loopTime >= Ts)
    {

      //printf("Mode: %i, Dim: %i, Red: %i\n", active_states[MODE], active_states[DIM], active_states[RED]);
      //printf("BPM: %i, Dim: %i, Red: %i\n", discostates[BPM], discostates[DIM], discostates[RED]);

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

  // init esp_now
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    //return;
  }

  // setup receiver info to channel 0
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, disco_address, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // add receiver
  if (esp_now_add_peer(&peerInfo) == ESP_OK) {
    Serial.println("Pair success");
  }
  else
  {
    Serial.println("Pair failed");
  }

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
      //printf("received");
      // Check that no errors occurred.
      if (packet.err == DMX_OK)
      {

        // dmx_read(dmx_num, data, packet.size);
        // add 1 to dmx size since first byte = NULL
        dmx_read_offset(dmx_num, dmx_start_addr, dmx_data, dmx_size+1);
        //Serial.println("data received");

        // send data to discoball (use pointer to discostates array and define length of array)
        esp_err_t send_status = esp_now_send(0, (uint8_t *) &discostates, disco_dmx_size);
        
        if (send_status == ESP_ERR_ESPNOW_NOT_FOUND){
          printf("Peer not found\n");
        } 
        // else {
        //   printf("Error code: %i\n", send_status);
        // }
        
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