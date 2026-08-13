#include <Arduino.h>

// own libraries
#include "led_functions.h"
#include "web_ui.h"

// other libraries
#include "esp_dmx.h"
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

// Sampling time (Ts)
#define Ts 15

// max numbers for settings
#define MAXCOLORS 10

// communication to discoball
// address to send data to: A8:42:E3:8D:B8:01
uint8_t send_to_address[] = {0xA8, 0x42, 0xE3, 0x8D, 0xB8, 0x01};
uint8_t newMACAddress[] = {0xA8, 0x42, 0xE3, 0x8D, 0xB8, 0x05};

// number of modes
const int num_modes = 12; // to be defined
const float mode_selector = ceil(256.0 / num_modes);

// scanner number
const int scanner_number = 0;

// setup DMX port and pins
const dmx_port_t dmx_num = DMX_NUM_2;
// DMX start address
const int dmx_start_addr = 0;
// DMX size (number of addresses)c
const int num_used_scanners = 6;
const int num_total_scanners = 12;
const int channels_per_scanner = 16;

// scanner to use for checking states
const int state_scanner = 11; // 12

// used dmx channels (except for scanner 12)
const int dmx_size_total = num_total_scanners*channels_per_scanner;
const int dmx_size_used = num_used_scanners*channels_per_scanner;

// define dmx received data
uint8_t dmx_data[dmx_size_total + 1];
uint8_t received_states[dmx_size_total][4];

// used states (numbers of channels + boolean on used scanner)
uint8_t used_states[channels_per_scanner + num_used_scanners];

// DMX pins (UART)
const int tx_pin = 17;
const int rx_pin = 16;
const int rts_pin = 21;

// define active states (are used by the lights)
const uint8_t MODE = 0;
const uint8_t BPM = 1;
const uint8_t DIM = 2;
const uint8_t DIMMER = 4;
const uint8_t RED = 5;
const uint8_t GREEN = 6;
const uint8_t BLUE = 7;
const uint8_t EXTRA1 = 8;
const uint8_t EXTRA2 = 9;
uint8_t active_states[16];

// previous mode for non direct switching
// time should be on same mode (in ms)
const float mode_change_time = 1000.0;
float start_time_mode; 
uint8_t new_mode;

// set up the different cores
TaskHandle_t ControllerTask;
TaskHandle_t LEDTask;
TaskHandle_t WebTask;

// bottom post y start and end (bottom to top)
float y_b1 = -0.17;
float y_b2 = 0.05;

// top post y start and end (bottom to top)
float y_t1 = y_b2;
float y_t2 = 0.27;

// x positions start & diff
float x_1 = 0.2;
float x_d = (0.5 - x_1) / 4;

// bottom letter (bottom to top)
float yl_b1 = -0.3;
float yl_b2 = -0.15;

// top letter (bottom to top)
float yl_t1 = -0.15;
float yl_t2 = 0;
// for the R
float yl_t3 = (yl_t1 / 2);

// start position 0, only diff
// letter width
float xl_d1 = x_d;
// distance from letter to letter
float xl_d2 = xl_d1 / 4;

// define letter x positions, otherwise to messy
// defined from left to right
float xk_1 = -2.0f * xl_d1 - 1.5f * xl_d2;
float xk_2 = -1.0f * xl_d1 - 1.5f * xl_d2;
float xr_1 = -1.0f * xl_d1 - 0.5f * xl_d2;
float xr_2 = -0.5f * xl_d2;
float xv_1 = 0.5f * xl_d2;
float xv_2 = xl_d1 / 4.0f + 0.5f * xl_d2;
float xv_3 = xl_d1 * 2.0f / 4.0f + 0.5f * xl_d2;
float xv_4 = xl_d1 * 3.0f / 4.0f + 0.5f * xl_d2;
float xv_5 = xl_d1 + 0.5f * xl_d2;
float xn_1 = xl_d1 + 1.5f * xl_d2;
float xn_2 = xl_d1 * 3.0f / 2.0f + 1.5f * xl_d2;
;
float xn_3 = xl_d1 * 2.0f + 1.5f * xl_d2;
;
// add whitspace 
float add_x[] = {x_d/2.0f, x_d/2.0f};
float add_y[] = {0.05, 0.05};

// define relative start and end position of the sides
float start_pos_x[] = {-x_1, -x_1, -x_1 - 1 * x_d, -x_1 - 1 * x_d, -x_1 - 2 * x_d, -x_1 - 2 * x_d, -x_1 - 3 * x_d, -x_1 - 3 * x_d, -x_1 - 4 * x_d, -x_1 - 4 * x_d,
                       x_1, x_1, x_1 + 1 * x_d, x_1 + 1 * x_d, x_1 + 2 * x_d, x_1 + 2 * x_d, x_1 + 3 * x_d, x_1 + 3 * x_d, x_1 + 4 * x_d, x_1 + 4 * x_d,
                       xk_1, xk_1, xk_2, xk_1, xr_1, xr_1, xr_1, xr_2, xr_1,
                       xv_1, xv_2, xv_3, xv_4, xn_1, xn_1, xn_1, xn_2, xn_3, xn_3};
float start_pos_y[] = {y_b1, y_t1, y_b1, y_t1, y_b1, y_t1, y_b1, y_t1, y_b1, y_t1,
                       y_b1, y_t1, y_b1, y_t1, y_b1, y_t1, y_b1, y_t1, y_b1, y_t1,
                       yl_b1, yl_t1, yl_t2, yl_b2, yl_b1, yl_t1, yl_t2, yl_t3, yl_b2,
                       yl_t2, yl_b2, yl_b1, yl_t1, yl_b1, yl_t1, yl_t2, yl_b2, yl_b1, yl_t1};
float end_pos_x[] = {-x_1, -x_1, -x_1 - 1 * x_d, -x_1 - 1 * x_d, -x_1 - 2 * x_d, -x_1 - 2 * x_d, -x_1 - 3 * x_d, -x_1 - 3 * x_d, -x_1 - 4 * x_d, -x_1 - 4 * x_d,
                     x_1, x_1, x_1 + 1 * x_d, x_1 + 1 * x_d, x_1 + 2 * x_d, x_1 + 2 * x_d, x_1 + 3 * x_d, x_1 + 3 * x_d, x_1 + 4 * x_d, x_1 + 4 * x_d,
                     xk_1, xk_1, xk_1, xk_2, xr_1, xr_1, xr_2, xr_1, xr_2,
                     xv_2, xv_3, xv_4, xv_5, xn_1, xn_1, xn_2, xn_3, xn_3, xn_3};
float end_pos_y[] = {y_b2, y_t2, y_b2, y_t2, y_b2, y_t2, y_b2, y_t2, y_b2, y_t2,
                     y_b2, y_t2, y_b2, y_t2, y_b2, y_t2, y_b2, y_t2, y_b2, y_t2,
                     yl_b2, yl_t2, yl_t1, yl_b1, yl_b2, yl_t2, yl_t3, yl_t1, yl_b1,
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

// web UI for troubleshooting: plots pixel_pos / live color and can trigger
// a mode directly, bypassing DMX input
WebUI webUI(LED, "Karavaan-Main", "ledtest123");

// the LED positions are defined in the setup loop
// can only declare variables in global space

void process_data()
{

    // check the change of the states
    for (int j = 0; j < channels_per_scanner; j++)
    {
        // shift received states in time
        received_states[state_scanner * channels_per_scanner + j][3] = received_states[state_scanner * channels_per_scanner + j][2];
        received_states[state_scanner * channels_per_scanner + j][2] = received_states[state_scanner * channels_per_scanner + j][1];
        received_states[state_scanner * channels_per_scanner + j][1] = received_states[state_scanner * channels_per_scanner + j][0];
        // get new received state
        received_states[state_scanner * channels_per_scanner + j][0] = dmx_data[state_scanner * channels_per_scanner + j + 1];

        int other_side = j < channels_per_scanner/2 ? channels_per_scanner/2 : -channels_per_scanner/2;

        // if (changed 3 times || if changed to different than other side)
        // && sum of 3 changes > 2
        int sum_changes = abs(received_states[state_scanner * channels_per_scanner + j][3] - received_states[state_scanner * channels_per_scanner + j][2]) + 
                          abs(received_states[state_scanner * channels_per_scanner + j][2] - received_states[state_scanner * channels_per_scanner + j][1]) +
                          abs(received_states[state_scanner * channels_per_scanner + j][1] - received_states[state_scanner * channels_per_scanner + j][0]);
        if (((received_states[state_scanner * channels_per_scanner + j][3] != received_states[state_scanner * channels_per_scanner + j][2] &&
              received_states[state_scanner * channels_per_scanner + j][2] != received_states[state_scanner * channels_per_scanner + j][1] &&
              received_states[state_scanner * channels_per_scanner + j][1] != received_states[state_scanner * channels_per_scanner + j][0]) ||
             (received_states[state_scanner * channels_per_scanner + j][0] != used_states[j + other_side] &&
              received_states[state_scanner * channels_per_scanner + j][1] != received_states[state_scanner * channels_per_scanner + j][0])) &&
              (sum_changes > 2))
            {

                used_states[j] = received_states[state_scanner * channels_per_scanner + j][0];

            }

    }

    // Loop through all scanners to see which are active
    for (int i = 0; i < num_used_scanners; i++)
    {
        // set to false until value > 0 is found
        bool scanner_used = false;

        for (int j = 0; j < channels_per_scanner; j++)
        {
            scanner_used = scanner_used || dmx_data[i * channels_per_scanner + j + 1] > 0;
        }

        // set the scanner correct in used states
        used_states[channels_per_scanner + i] = scanner_used;
        
    }

}

void set_states()
{

  // reset counter on mode
  if (active_states[MODE] != static_cast<uint8_t>(floor(used_states[MODE]/mode_selector)))
  {
    LED.resetCounters();
  }

  // copy the first channels_per_scanner used_states to active_states
  for(int i = 0; i < channels_per_scanner; i++)
  {
    active_states[i] = used_states[i];
  }

  // use first state te select the 
  active_states[MODE] = static_cast<uint8_t>(floor(static_cast<float>(used_states[MODE])/mode_selector));

  if(used_states[channels_per_scanner + scanner_number] == 0)
  {
    active_states[DIM] = 0;
  }

  // set a BPM of at least 1
  active_states[BPM] = active_states[BPM] > 1 ? active_states[BPM] : 1;

}

void setColor()
{

  float red = static_cast<float>(active_states[RED]);
  float green = static_cast<float>(active_states[GREEN]);
  float blue = static_cast<float>(active_states[BLUE]);
  float white = 0;

  // if red green and blue are almost equal, select white
  if (abs(red - green) + abs(green - blue) < 10)
  {
    white = 255;
    red = 0;
    green = 0;
    blue = 0;
  }

  // else normalize red green and blue to 255
  else
  {
    float max_color = red;
    max_color = green > max_color ? green : max_color;
    max_color = blue > max_color ? blue : max_color;

    // define normalization factor
    // divide all colors by max color and multiply by 255
    float max_color_f = max_color;
    red = red / max_color * 255.0f;
    green = green / max_color * 255.0f;
    blue = blue / max_color * 255.0f;
  }

  uint8_t white_i = static_cast<uint8_t>(white);
  uint8_t red_i = static_cast<uint8_t>(red);
  uint8_t green_i = static_cast<uint8_t>(green);
  uint8_t blue_i = static_cast<uint8_t>(blue);

  LED.changeColor(white_i, red_i, green_i, blue_i);
}

void setmode()
{
  LED.freqdiv = 1;
  switch (active_states[MODE])
  {
  case 0: // DONE
  { //
    // clusters are defined by a pole or full letter
    uint8_t clusters[] = {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 4, 5, 4, 6};
    uint8_t num_clusters = sizeof(clusters) / sizeof(uint8_t);
    float ramp_time = 0.02;
    float fade_time = mapValue(0, 255, 0, 5, active_states[DIMMER]);
    float on_time;
    float on_chance;
    if (active_states[EXTRA1] < 85)
    {
      on_time = 1;
      on_chance = 1 - mapValue(0, 84, 0, 0.8, active_states[EXTRA1]);
    }
    else if (active_states[EXTRA1] < 171)
    {
      on_time = 1 - mapValue(85, 170, 0, 0.9, active_states[EXTRA1]);
      on_chance = 0.2;
    }
    else
    {
      on_time = 0.1;
      on_chance = 1 - mapValue(171, 255, 0.8, 0, active_states[EXTRA1]);
    }
    LED.strobo(0, num_clusters, clusters, ramp_time, on_time, on_chance, fade_time);
    break;
  }

  case 1: // DONE
  { // Cluster alternate in up down fashion
    bool clusters1[] = {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0,
                             1, 0, 0, 1, 1, 0, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0};
    bool clusters2[] = {0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1,
                             0, 1, 1, 0, 0, 1, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1};
    float fadetime = mapValue(0, 255, 0, 5, active_states[DIMMER]);
    float on_time = mapValue(0, 255, 0.75, 0.1, active_states[EXTRA1]);
    {
      LED.alternateClusters(clusters1, clusters2, fadetime, on_time);
    }
    break;
  }

  case 2: // DONE
  { // increase number of pixels and colors gradually
    float fadetime = mapValue(0, 255, 0, 5, active_states[DIMMER]);
    // flash chance between 5 and 75 %
    uint8_t flash_chance = (uint8_t)mapValue(0, 255, 5, 50, active_states[EXTRA1]);
    uint8_t num_colors = 1;
    if (active_states[EXTRA1]>100)
    {
      num_colors = (uint8_t)mapValue(101, 255, 1, 3, active_states[EXTRA1]);
    }
    LED.flashingPixels(0, flash_chance, fadetime, num_colors);
    break;
  }

  case 3: // DONE
  {
    // Lines move in carthesion up or down
    LED.freqdiv = 4;
    float linewidth = 0.05;
    float fadetime = mapValue(0, 255, 0, 5, active_states[DIMMER]);
    float slider_mapper = mapValue(0, 255, 0.5, 6.49, active_states[EXTRA1]);
    uint8_t number_of_lines = (uint8_t)round(slider_mapper);
    int direction = floor(slider_mapper) != round(slider_mapper) ? 2 : 4;
    LED.movingLines(number_of_lines, direction, fadetime, linewidth);
    break;
  }

  case 4: // DONE
  {
    // Lines move in carthesion to mimic rotation
    LED.freqdiv = 4;
    float linewidth = 1.0f/40.0f;
    float fadetime = mapValue(0, 255, 0, 5, active_states[DIMMER]);
    float slider_mapper = mapValue(0, 255, 0.5, 4.49, active_states[EXTRA1]);
    uint8_t number_of_lines = (uint8_t)round(slider_mapper);
    int direction = floor(slider_mapper) != round(slider_mapper) ? 1 : 3;
    LED.movingLines(number_of_lines, direction, fadetime, linewidth);
    break;
  }

  case 5: // DONE
  { // rainbow light switching
    float blend_level = mapValue(0, 255, 0, 1, active_states[EXTRA1]);
    int direction = 1; 
    LED.rainbow(blend_level, direction);
    break;
  }
  }
}

// receives control values from the web UI and feeds them into the same
// used_states[] array set_states() already reads from DMX, so the
// existing setmode()/setColor() logic runs unchanged
void onWebSet(uint8_t mode, uint8_t bpm, uint8_t dim, uint8_t dimmer,
              uint8_t red, uint8_t green, uint8_t blue,
              uint8_t extra1, uint8_t extra2)
{
  used_states[MODE] = static_cast<uint8_t>(mode * mode_selector);
  used_states[BPM] = bpm;
  used_states[DIM] = dim;
  used_states[DIMMER] = dimmer;
  used_states[RED] = red;
  used_states[GREEN] = green;
  used_states[BLUE] = blue;
  used_states[EXTRA1] = extra1;
  used_states[EXTRA2] = extra2;

  // mark the scanner as "used" so set_states() doesn't zero the dimmer
  used_states[channels_per_scanner + scanner_number] = 1;
}

// Task for handling the LEDs on core 1
void LightsTaskcode(void *pvParameters)
{

  // define LED positions
  LED.definePositions_carthesian(start_pos_x, start_pos_y, end_pos_x, end_pos_y);
  LED.add_sidespace(add_x[0], add_x[1], add_y[0], add_y[1]);

  // reset for stability
  LED.resetPixels();

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

      // set the states correctly
      set_states();

      // set dimmer value (should be between 0 and 1)
      LED.setDimmer((static_cast<float>(active_states[DIM])) / 255);

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

  // Set device as a Wi-Fi Station, and also host an AP for the web UI
  WiFi.mode(WIFI_AP_STA);

  // set mac address
  esp_wifi_set_mac(WIFI_IF_STA, &newMACAddress[0]);

  // print the mac address
  Serial.println(WiFi.macAddress());

  // disconnect wifi
  WiFi.disconnect();

  // init esp_now
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    // return;
  }

  // setup receiver info to channel 0
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, send_to_address, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // add receiver
  if (esp_now_add_peer(&peerInfo) == ESP_OK)
  {
    Serial.println("Pair success");
  }
  else
  {
    Serial.println("Pair failed");
  }

  // start the web UI's AP + HTTP routes from here too: WiFi.mode()/
  // softAP()/esp_now_init() share internal driver state and are not safe
  // to call concurrently from another task, so this task stays the only
  // one that ever mutates WiFi driver state. WebTaskcode only ever calls
  // handleClient() (plain socket I/O), which is safe to run in parallel.
  webUI.onSet(onWebSet);
  webUI.begin();

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
      // printf("received");
      //  Check that no errors occurred.
      if (packet.err == DMX_OK)
      {

        // dmx_read(dmx_num, data, packet.size);
        // add 1 to dmx size since first byte = NULL
        dmx_read_offset(dmx_num, dmx_start_addr, dmx_data, dmx_size_total + 1);
        // Serial.println("data received");

        // process the data
        process_data();

        // send data to discoball (use pointer to discostates array and define length of array)
        esp_err_t send_status = esp_now_send(0, (uint8_t *)&used_states, channels_per_scanner + num_used_scanners);

        if (send_status == ESP_ERR_ESPNOW_NOT_FOUND)
        {
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

    // task delay for stability
    vTaskDelay(1);

  }
}

// serves the troubleshooting web UI on core 0, alongside ControllerTask
void WebTaskcode(void *pvParameters)
{
  for (;;)
  {
    webUI.handle();
    vTaskDelay(2);
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

  // create a task that serves the web UI, executed on core 0
  xTaskCreatePinnedToCore(
      WebTaskcode, /* Task function. */
      "WebTask",   /* name of task. */
      8000,        /* Stack size of task */
      NULL,        /* parameter of the task */
      1,           /* priority of the task */
      &WebTask,    /* Task handle to keep track of created task */
      0);          /* pin task to core 0 */
  delay(500);
}

// loop can be left empty, tasks are used
void loop()
{
}