#include <Arduino.h>

// own libraries
#include "led_functions.h"

// other libraries
#include "esp_dmx.h"
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

// Sampling time (Ts)
#define Ts 1000

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

// set up the different cores
TaskHandle_t ControllerTask;
TaskHandle_t LEDTask;

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

        // if changed 2 times, change used state
        if ((received_states[state_scanner * channels_per_scanner + j][3] != received_states[state_scanner * channels_per_scanner + j][2] &&
             received_states[state_scanner * channels_per_scanner + j][2] != received_states[state_scanner * channels_per_scanner + j][1] &&
             received_states[state_scanner * channels_per_scanner + j][1] != received_states[state_scanner * channels_per_scanner + j][0]) ||
            (received_states[state_scanner * channels_per_scanner + j][0] != used_states[j + other_side] &&
             received_states[state_scanner * channels_per_scanner + j][1] != received_states[state_scanner * channels_per_scanner + j][0]))
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

        // print the states
        for (int i = 0; i < channels_per_scanner + num_used_scanners; i++)
        {
            printf("%i, ", used_states[i]);
        }
        printf("\n");

    }
  }

}

// handle DMX on core 0
void ControllerTaskcode(void *pvParameters)
{

  // First, use the default DMX configuration...
  dmx_config_t config = DMX_CONFIG_DEFAULT;
  // set start address to 0
  config.dmx_start_address = dmx_start_addr;

  // ...install the DMX driver...
  dmx_driver_install(dmx_num, &config, DMX_INTR_FLAGS_DEFAULT);

  dmx_set_pin(dmx_num, tx_pin, rx_pin, rts_pin);

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