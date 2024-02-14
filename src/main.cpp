#include <Arduino.h>

// own libraries
#include "led_functions.h"

// other libraries
#include "esp_dmx.h"
#include<WiFi.h>
#include<esp_wifi.h>
#include<esp_now.h>

// Sampling time (Ts)
#define Ts 12

// max numbers for settings
#define MAXCOLORS 10
#define MAXMODES 16 // to be defined

// setup DMX port and pins
const dmx_port_t dmx_num = DMX_NUM_2;
// DMX start address
const int dmx_start_addr = 0;
// number of states for LED and discoball
const int led_dmx_size = 96; // muliples of 8
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
uint8_t newMACAddress[] = {0xA8, 0x42, 0xE3, 0x8D, 0xB8, 0x05};

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
const uint8_t DIMMER = 5;
const uint8_t EXTRA1 = 6;
const uint8_t EXTRA2 = 7;
const uint8_t MODE = 8;
uint8_t active_states[9];

// set up the different cores
TaskHandle_t ControllerTask;
TaskHandle_t LEDTask;

// bottom post y start and end (bottom to top)
float y_b1 = -0.17;
float y_b2 = 0.05;

// top post y start and end (bottom to top)
float y_t1 = y_b2;
float y_t2 = 0.27;

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
float yl_t3 = (yl_t1/2);

// start position 0, only diff
// letter width
float xl_d1 = x_d;
// distance from letter to letter
float xl_d2 = xl_d1/4;

// define letter x positions, otherwise to messy
// defined from left to right
float xk_1 = -2.0f*xl_d1-1.5f*xl_d2;
float xk_2 = -1.0f*xl_d1-1.5f*xl_d2;
float xr_1 = -1.0f*xl_d1-0.5f*xl_d2;
float xr_2 = -0.5f*xl_d2;
float xv_1 = 0.5f*xl_d2;
float xv_2 = xl_d1/4.0f + 0.5f*xl_d2;
float xv_3 = xl_d1*2.0f/4.0f + 0.5f*xl_d2;
float xv_4 = xl_d1*3.0f/4.0f + 0.5f*xl_d2;
float xv_5 = xl_d1 + 0.5f*xl_d2;
float xn_1 = xl_d1 + 1.5f*xl_d2;
float xn_2 = xl_d1*3.0f/2.0f + 1.5f*xl_d2;;
float xn_3 = xl_d1*2.0f + 1.5f*xl_d2;;


// define relative start and end position of the sides
float start_pos_x[] = {-x_1, -x_1,  -x_1-1*x_d, -x_1-1*x_d, -x_1-2*x_d, -x_1-2*x_d, -x_1-3*x_d, -x_1-3*x_d, -x_1-4*x_d, -x_1-4*x_d,
                        x_1,  x_1,   x_1+1*x_d,  x_1+1*x_d,  x_1+2*x_d,  x_1+2*x_d,  x_1+3*x_d,  x_1+3*x_d,  x_1+4*x_d,  x_1+4*x_d,
                        xk_1, xk_1, xk_2, xk_1, xr_1, xr_1, xr_1, xr_2, xr_1,
                        xv_1, xv_2, xv_3, xv_4, xn_1, xn_1, xn_1, xn_2, xn_3, xn_3};
float start_pos_y[] = {y_b1, y_t1, y_b1, y_t1, y_b1, y_t1, y_b1, y_t1, y_b1, y_t1,
                       y_b1, y_t1, y_b1, y_t1, y_b1, y_t1, y_b1, y_t1, y_b1, y_t1,
                       yl_b1, yl_t1, yl_t2, yl_b2, yl_b1, yl_t1, yl_t2, yl_t3, yl_b2,
                       yl_t2, yl_b2, yl_b1, yl_t1, yl_b1, yl_t1, yl_t2, yl_b2, yl_b1, yl_t1};
float end_pos_x[] = {-x_1, -x_1,  -x_1-1*x_d, -x_1-1*x_d, -x_1-2*x_d, -x_1-2*x_d, -x_1-3*x_d, -x_1-3*x_d, -x_1-4*x_d, -x_1-4*x_d,
                      x_1,  x_1,   x_1+1*x_d,  x_1+1*x_d,  x_1+2*x_d,  x_1+2*x_d,  x_1+3*x_d,  x_1+3*x_d,  x_1+4*x_d,  x_1+4*x_d,
                      xk_1, xk_1, xk_1, xk_2, xr_1, xr_1, xr_2, xr_1, xr_2,
                      xv_2, xv_3, xv_4, xv_5, xn_1, xn_1, xn_2, xn_3, xn_3, xn_3};
float end_pos_y[] = {y_b2, y_t2, y_b2, y_t2, y_b2, y_t2, y_b2, y_t2, y_b2, y_t2,
                     y_b2, y_t2, y_b2, y_t2, y_b2, y_t2, y_b2, y_t2, y_b2, y_t2,
                     yl_b2, yl_t2, yl_t1, yl_b1, yl_b2, yl_t2, yl_t1, yl_t1, yl_b1,
                     yl_t1, yl_b1, yl_b2, yl_t2, yl_b2, yl_t2, yl_t1, yl_b1, yl_b2, yl_t2};

// define number of leds per side
uint16_t LEDsPerSide[] = {18, 18, 18, 18, 18, 18, 18, 18, 18, 18,
                          18, 18, 18, 18, 18, 18, 18, 18, 18, 18,
                          12, 13, 17, 17, 12, 12, 13, 13, 17,
                          12, 13, 13, 12, 12, 13, 13, 13, 13, 12};
// get number of sides
uint8_t numSides = sizeof(LEDsPerSide) / sizeof(uint16_t);
uint8_t sidesPerPin[] = {10, 10, 9, 10};
uint8_t LEDPins[] = {26, 25, 33, 32};
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

void setColor(){

  float red = static_cast<float>(active_states[RED]);
  float green = static_cast<float>(active_states[GREEN]);
  float blue = static_cast<float>(active_states[BLUE]);
  float white = 0;

  // if red green and blue are almost equal, select white
  if (abs(red - green) + abs(green - blue) < 10){
    white = 255;
    red = 0;
    green = 0;
    blue = 0;
  } 

  // else normalize red green and blue to 255
  else {
    float max_color = red;
    max_color = green > max_color ? green : max_color;
    max_color = blue > max_color ? blue : max_color;
    
    // define normalization factor
    // divide all colors by max color and multiply by 255
    float max_color_f = max_color;
    red = red/max_color*255.0f;
    green = green/max_color*255.0f;
    blue = blue/max_color*255.0f;

  }

  uint8_t white_i = static_cast<uint8_t>(white);
  uint8_t red_i = static_cast<uint8_t>(red);
  uint8_t green_i = static_cast<uint8_t>(green);
  uint8_t blue_i = static_cast<uint8_t>(blue);

  LED.changeColor(white_i, red_i, green_i, blue_i);

}

void setmode(){
  LED.freqdiv = 1;
  switch (active_states[MODE])
    {
    case 0: {
      uint8_t clusters[] = {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 4, 5, 4, 6};
      uint8_t num_clusters = sizeof(clusters)/sizeof(uint8_t);
      float ramp_time = 0.02;
      float fade_time = mapValue(0, 255, 0, 5, active_states[DIMMER]);
      float on_time = 1-mapValue(0, 255, 0, 1, active_states[EXTRA1]);
      float on_chance = 1-mapValue(0, 255, 0, 1, active_states[EXTRA2]);
      LED.strobo(0, num_clusters, clusters, ramp_time, on_time, on_chance, fade_time);
      break; 
      }

    case 1: {
      uint8_t clusters[] =      {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 4, 5, 4, 6};
      uint8_t cluster_order[] = {4, 3, 2, 1, 0, 10, 11, 12, 13, 5, 6, 7, 8, 9};
      uint8_t num_clusters = sizeof(clusters)/sizeof(uint8_t);
      float fade_time = mapValue(0, 255, 0, 5, active_states[DIMMER]);
      int direction = active_states[EXTRA1] < 128 ? 1 : -1;
      LED.moveClockwise(num_clusters, clusters, cluster_order, direction, fade_time);
      break; 
      }

    case 2: {
      // between 0 and 0.99
      float fadetime = mapValue(0, 255, 0, 5, active_states[DIMMER]);
      // flash chance between 5 and 75 %
      uint8_t flash_chance = (uint8_t)mapValue(0, 255, 5, 50, active_states[EXTRA1]);
      uint8_t num_colors = (uint8_t)mapValue(0, 255, 1, 3, active_states[EXTRA2]);
      LED.flashingPixels(0, flash_chance, fadetime, num_colors);
      break; 
      }

    case 3: {
      float width_angle = 2.0f*PI/36.0f;
      int direction = active_states[EXTRA2] < 128 ? 1 : -1;
      float fadetime = mapValue(0, 255, 0, 5, active_states[DIMMER]);
      if (active_states[EXTRA1] < 85){
        LED.oneColorRotation(1, width_angle, direction, fadetime);
      } else if (active_states[EXTRA1] >= 85 && active_states[EXTRA1] < 171) {
        LED.oneColorRotation(2, width_angle, direction, fadetime);
      } else {
        LED.twoColorRotation(1, width_angle, direction, fadetime);
      }
      break;
      }

    case 4: {
      float circle_width = 0.08;
      float clip_radius = 1;
      float fadetime = mapValue(0, 255, 0, 5, active_states[DIMMER]);
      uint8_t num_circles = static_cast<uint8_t>(round(mapValue(0, 255, 1, 3, active_states[EXTRA1])));
      int direction = active_states[EXTRA2] < 128 ? 1 : -1;
      LED.movingCircles(num_circles, circle_width, direction, fadetime, clip_radius);
      break;
      }

    case 5: {
      float fadetime = mapValue(0, 255, 0, 5, active_states[DIMMER]);
      float block_size = mapValue(0, 255, 0.1, 0.3, active_states[EXTRA1]);
      float move_width = mapValue(0, 255, 1, 4, active_states[EXTRA2]);
      float y_range[] = {-0.3, 0.3};
      LED.movingBlock(block_size, fadetime, move_width, y_range);
      break;
      }

    case 6: {
      // use clusters of a pole of a full letter
      bool clusters1_opt1[] = {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 
                          1, 0, 0, 1, 1, 0, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0};
      bool clusters2_opt1[] = {0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1,
                          0, 1, 1, 0, 0, 1, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1};
      bool clusters1_opt2[] = {0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 
                          1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0};
      bool clusters2_opt2[] = {1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0,
                          0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1};
      float fadetime = mapValue(0, 255, 0, 5, active_states[DIMMER]); 
      float on_time = mapValue(0, 255, 0.2, 0.5, active_states[EXTRA1]);
      if (active_states[EXTRA2] < 128){
        LED.alternateClusters(clusters1_opt1, clusters2_opt1, fadetime, on_time);
      } else {
        LED.alternateClusters(clusters1_opt2, clusters2_opt2, fadetime, on_time);
      }
      break;
      }

    case 7: {
      // do nothing.. the button is broken
      }

    case 8: {
      // use clusters of a pole of a full letter
      float linewidth = 0.05;
      float fadetime = mapValue(0, 255, 0, 5, active_states[DIMMER]); 
      uint8_t direction = mapValue(0, 255, 1, 4, active_states[EXTRA1]);
      uint8_t number_of_lines = mapValue(0, 255, 1, 4, active_states[EXTRA2]);
      LED.movingLines(number_of_lines, direction, fadetime, linewidth);
      break;
      }

    case 9: {
      // use clusters of a pole of a full letter
      float fadetime = mapValue(0, 255, 0, 5, active_states[DIMMER]); 
      float updown_time = mapValue(0, 255, 0.5, 1, active_states[EXTRA1]);
      float phase = mapValue(0, 255, -1, 1, active_states[EXTRA2]);
      float line_width = 0.15;
      float y_range[] = {yl_b1, y_t2};
      LED.updownPositionBased(updown_time, fadetime, phase, line_width, y_range);
      break;
      }

    case 10: {
      // use clusters of a pole of a full letter
      uint8_t clusters[] = {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 4, 5, 4, 6};
      uint8_t num_clusters = sizeof(clusters)/sizeof(uint8_t);
      // between 0 and 0.99
      float fadetime = mapValue(0, 255, 0, 5, active_states[DIMMER]);
      // between 1 and 4
      uint8_t num_pixels = (uint8_t)mapValue(0, 255, 1, 10, active_states[EXTRA1]);
      int direction = active_states[EXTRA2] < 128 ? 1 : -1;
      float bandwidth = 1;
      LED.movingPixel(0, num_clusters, clusters, direction, fadetime, num_pixels, bandwidth);
      break; 
      }

    case 11: {
      float fadetime = mapValue(0, 255, 0, 5, active_states[DIMMER]);
      float line_size = mapValue(0, 255, 0.5, 0.5, active_states[EXTRA1]);
      bool inverse = active_states[EXTRA2] < 128 ? true : false;
      LED.heartbeat(line_size, fadetime, inverse);
      break;
      }
    }
}


// Task for handling the LEDs on core 1
void LightsTaskcode(void *pvParameters)
{

  // define LED positions
  LED.definePositions_carthesian(start_pos_x, start_pos_y, end_pos_x, end_pos_y);

  // reset for stability
  LED.resetPixels();

  // TIME VARIABLES
  // Time spent in the main loop
  int loopTime = 0;

  // active_states[MODE] = 2;

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
      setColor();

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

  // set mac address
  esp_wifi_set_mac(WIFI_IF_STA, &newMACAddress[0]);

  // print the mac address
  Serial.println(WiFi.macAddress());

  // disconnect wifi
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

    // esp_err_t send_status = esp_now_send(0, (uint8_t *) &discostates, disco_dmx_size);
    
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