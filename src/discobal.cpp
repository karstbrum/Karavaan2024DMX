#include <Arduino.h>

// own libraries
#include "led_functions.h"

// communication libraries
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

// Sampling time (Ts)
#define Ts 15

// set mac adress (=needed when switching esp boards)
uint8_t newMACAddress[] = {0xA8, 0x42, 0xE3, 0x8D, 0xB8, 0x01};

// number of modes
const int num_modes = 12; // to be defined
const float mode_selector = ceil(256.0 / num_modes);

// scanner number
const int scanner_number = 1;
const int scanner_number_motor = 2;

// DMX size (number of addresses)c
const int num_used_scanners = 6;
const int channels_per_scanner = 16;

const int dmx_size_used = num_used_scanners*channels_per_scanner;

// used states (numbers of channels + boolean on used scanner)
uint8_t used_states[dmx_size_used];

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
uint8_t active_states[channels_per_scanner];

// check if motor should be on
bool motor_on;

// define tasks (multicore)
TaskHandle_t LEDTask;
TaskHandle_t ControllerTask;

// define start angle of first and second strip
float a1 = PI / 12 * 1;
float a2 = PI + a1;

// define angle difference between clusters
float da = PI / 12 * 2.5;

// define relative start and end angles
// the diameter is normalized to 1
float start_a[] = {a1, a1 + da, a1 + 2 * da, a1 + 3 * da, a2, a2 + da, a2 + 2 * da, a2 + 3 * da};
float end_a[] = {a1 + da, a1 + 2 * da, a1 + 3 * da, a1 + 4 * da, a2 + da, a2 + 2 * da, a2 + 3 * da, a2 + 4 * da};
float l_sides[] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};

// 288 lights divided in 8 segments
uint16_t LEDsPerSide[] = {36, 36, 36, 36, 36, 36, 36, 36};
uint8_t numSides = sizeof(LEDsPerSide) / sizeof(uint16_t);
uint8_t sidesPerPin[] = {8};
uint8_t LEDPins[] = {33};
uint8_t numPins = sizeof(LEDPins);
Pixels LED(numSides, LEDsPerSide, numPins, sidesPerPin, LEDPins, Ts);

// output pin for motor
uint8_t motor_pin = 32;

// level to turn motor on
uint8_t motor_on_level = 100;

// pointers to objects
Pixels *LED_pointer = &LED;

// TIME VARIABLES
// Time spent in the main loop
int loopTime = 0;

// data receive function
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len)
{

  // map data to discostates
  for (int i = 0; i < dmx_size_used; i++)
  {
    used_states[i] = incomingData[i];
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
  active_states[MODE] = static_cast<uint8_t>(floor(used_states[MODE]/mode_selector));

  if(used_states[channels_per_scanner + scanner_number] == 0)
  {
    active_states[DIM] = 0;
  }

  if(used_states[channels_per_scanner + scanner_number_motor] == 1)
  {
    motor_on = true;
  }
  else
  {
    motor_on = false;
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
  case 0:
  { //
    // use clusters of a pole of a full letter
    uint8_t clusters[] = {1, 1, 1, 1, 1, 1, 1, 1};
    uint8_t num_clusters = sizeof(clusters) / sizeof(uint8_t);
    float ramp_time = 0.02;
    float fade_time = mapValue(0, 255, 0, 5, active_states[DIMMER]);
    float on_time = 1 - mapValue(0, 255, 0, 1, active_states[EXTRA1]);
    float on_chance = 1 - mapValue(0, 255, 0, 1, active_states[EXTRA2]);
    LED.strobo(0, num_clusters, clusters, ramp_time, on_time, on_chance, fade_time);
    break;
  }

  case 1:
  { //
    // use clusters of a pole of a full letter
    uint8_t clusters[] = {1, 1, 1, 1, 1, 1, 1, 1};
    uint8_t cluster_order[] = {0, 1, 2, 3, 4, 5, 6, 7};
    uint8_t num_clusters = sizeof(clusters) / sizeof(uint8_t);
    float fade_time = mapValue(0, 255, 0, 5, active_states[DIMMER]);
    int direction = active_states[EXTRA1] < 128 ? 1 : -1;
    float cluster_length = 1 - mapValue(0, 255, 0, 1, active_states[EXTRA2]);
    LED.moveClockwise(num_clusters, clusters, cluster_order, direction, fade_time, cluster_length);
    break;
  }

  case 2:
  { //
    // between 0 and 0.99
    float fadetime = mapValue(0, 255, 0, 5, active_states[DIMMER]);
    // flash chance between 5 and 75 %
    uint8_t flash_chance = (uint8_t)mapValue(0, 255, 5, 50, active_states[EXTRA1]);
    uint8_t num_colors = (uint8_t)mapValue(0, 255, 1, 3, active_states[EXTRA2]);
    LED.flashingPixels(0, flash_chance, fadetime, num_colors);
    break;
  }

  case 3:
  { //
    float width_angle = 2.0f * PI / 100.0f;
    int direction = active_states[EXTRA2] < 128 ? 1 : -1;
    float fadetime = mapValue(0, 255, 0, 5, active_states[DIMMER]);
    if (active_states[EXTRA1] < 85)
    {
      LED.oneColorRotation(1, width_angle, direction, fadetime);
    }
    else if (active_states[EXTRA1] >= 85 && active_states[EXTRA1] < 171)
    {
      LED.oneColorRotation(2, width_angle, direction, fadetime);
    }
    else
    {
      LED.twoColorRotation(1, width_angle, direction, fadetime);
    }
    break;
  }

  case 4:
  {
    float circle_width = 0.08;
    float clip_radius = 1;
    float fadetime = mapValue(0, 255, 0, 5, active_states[DIMMER]);
    uint8_t num_circles = static_cast<uint8_t>(round(mapValue(0, 255, 1, 5, active_states[EXTRA1])));
    int direction = active_states[EXTRA2] < 128 ? 1 : -1;
    LED.movingCircles(num_circles, circle_width, direction, fadetime, clip_radius);
    break;
  }

  case 5:
  {
    float fadetime = mapValue(0, 255, 0, 5, active_states[DIMMER]);
    float block_size = mapValue(0, 255, 0.7, 0.8, active_states[EXTRA1]);
    float move_width = mapValue(0, 255, 1, 2, active_states[EXTRA2]);
    float y_range[] = {-0.6, 0.6};
    LED.movingBlock(block_size, fadetime, move_width, y_range);
    break;
  }

  case 6:
  {
    // use clusters of a pole of a full letter
    bool clusters1_opt1[] = {1, 1, 1, 1, 0, 0, 0, 0};
    bool clusters2_opt1[] = {0, 0, 0, 0, 1, 1, 1, 1};
    bool clusters1_opt2[] = {1, 1, 0, 0, 1, 1, 0, 0};
    bool clusters2_opt2[] = {0, 0, 1, 1, 0, 0, 1, 1};
    float fadetime = mapValue(0, 255, 0, 5, active_states[DIMMER]);
    float on_time = mapValue(0, 255, 0.2, 0.5, active_states[EXTRA1]);
    // select different clusters based on input
    if (active_states[EXTRA2] < 128)
    {
      LED.alternateClusters(clusters1_opt1, clusters2_opt1, fadetime, on_time);
    }
    else
    {
      LED.alternateClusters(clusters1_opt2, clusters2_opt2, fadetime, on_time);
    }
    break;
  }

  case 7:
  {
    // use clusters of a pole of a full letter
    float linewidth = 0.05;
    float fadetime = mapValue(0, 255, 0, 5, active_states[DIMMER]);
    uint8_t direction = mapValue(0, 255, 1, 4, active_states[EXTRA1]);
    uint8_t number_of_lines = mapValue(0, 255, 1, 4, active_states[EXTRA2]);
    LED.movingLines(number_of_lines, direction, fadetime, linewidth);
    break;
  }

  case 8:
  {
    // use clusters of a pole of a full letter
    float fadetime = mapValue(0, 255, 0, 5, active_states[DIMMER]);
    float updown_time = mapValue(0, 255, 0.5, 1, active_states[EXTRA1]);
    float phase = mapValue(0, 255, -0.1, 0.1, active_states[EXTRA2]);
    float line_width = 0.3;
    float y_range[] = {-0.5, 0.5};
    LED.updownPositionBased(updown_time, fadetime, phase, line_width, y_range);
    break;
  }

  case 9:
  {
    // use clusters of a pole of a full letter
    uint8_t clusters[] = {8};
    uint8_t num_clusters = sizeof(clusters) / sizeof(uint8_t);
    // between 0 and 5
    float fadetime = mapValue(0, 255, 0, 5, active_states[DIMMER]);
    // between 1 and 4
    uint8_t num_pixels = (uint8_t)mapValue(0, 255, 1, 10, active_states[EXTRA1]);
    int direction = active_states[EXTRA2] < 128 ? 1 : -1;
    float bandwidth = 1;
    LED.movingPixel(0, num_clusters, clusters, direction, fadetime, num_pixels, bandwidth);
    break;
  }

  case 10:
  {
    float fadetime = mapValue(0, 255, 0, 5, active_states[DIMMER]);
    float line_size = 0.5;
    bool inverse = active_states[EXTRA2] < 128 ? true : false;
    float pulse_time = 1 - mapValue(0, 255, 0, 0.75, active_states[EXTRA1]);
    LED.heartbeat(line_size, fadetime, inverse, pulse_time);
    break;
  }

  case 11:
  {
    float blend_level = mapValue(0, 255, 0, 1, active_states[EXTRA1]);
    int direction = active_states[EXTRA2] < 128 ? 1 : -1;
    LED.rainbow(blend_level, direction);
    break;
  }

  }
}

void setmotor()
{
  // if motor level is equal or greater than motor_on_level, turn motor on
  if (motor_on)
  {
    digitalWrite(motor_pin, HIGH);
  }
  else
  {
    digitalWrite(motor_pin, LOW);
  }
}

// Task for handling the LEDs on core 1
void LightsTaskcode(void *pvParameters)
{

  // set pinmode of discoball motor to output
  pinMode(motor_pin, OUTPUT);

  // define LED positions
  LED.definePositions_polar(start_a, end_a, l_sides);

  // reset for stability
  LED.resetPixels();

  // another option to have a timed loop is to use vTaskDelayUntil(), have to look into it first
  for (;;)
  {

    if (millis() - loopTime >= Ts)
    {

      loopTime = millis();

      // select the correct mode
      set_states();

      // set the motor
      setmotor();

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

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // set mac address
  esp_wifi_set_mac(WIFI_IF_STA, &newMACAddress[0]);

  // print the mac address
  Serial.println(WiFi.macAddress());

  // setup espnow
  WiFi.disconnect();
  esp_now_init();

  esp_now_register_recv_cb(OnDataRecv);

  // loop and read for DMX packets
  for (;;)
  {

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