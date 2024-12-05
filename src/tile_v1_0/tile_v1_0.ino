/*
 * L-Cube Tile Software
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <hardware/pio.h>
#include "differential_manchester.pio.h"
#include "lookup.h"
#include "crossing.h"
#include "sweeping.h"
#include "isr.h"
#include "sensor_config.h"
#include "config.h"
#include "mmath.h"
#include "file_system.h"
#include "positioning.h"
#include "i2c.h"
#include "bs_configuration.h"
#include <Adafruit_NeoPixel.h>
#include <Wire.h>


// class for ws2812 leds on top (neopixels)
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_LEDS, PIN_LEDS, NEO_GRB + NEO_KHZ800);

// - I2C ------------------------------------------------------------------------------------------
// i2c buffer which was received
char i2c_receive_buffer[CONFIG_SIZE + 1] = {0};
// i2c buffer that is sent to cube
char i2c_send_buffer[QUICK_CONFIG_DATA_BYTES + 1] = {0};
// i2c buffer that is sent for position updates
char i2c_update_data[UPDATE_SIZE + 1] = {0};
// i2c address of tile
uint8_t i2c_address = I2C_ADDRESS_OFFSET;
// flag for i2c message was received
uint8_t i2c_received = 0;

// - PHOTODIODES ----------------------------------------------------------------------------------
// flags to set in ISR and handle in loop()
uint8_t e_flag[MAX_PHOTODIODES] = { 0 };
// isr declarations
void (*isrs[MAX_PHOTODIODES])() = { isr_e0, isr_e1, isr_e2, isr_e3};
// data and envelope pins of the receiving sensors (photodiode+ts4231)
uint8_t photodiode_d_pin[MAX_PHOTODIODES] = {D_PINS};
uint8_t photodiode_e_pin[MAX_PHOTODIODES] = {E_PINS};
// rp2040 contains two pio blocks (pio0,pio1) with four state machines each which share a memory of 32 instructions
// we only use pio1 entirely -> pio0,sm0 is used by adafruit neopixel
PIO pio = pio1;
// indexes of state machines of pio1
uint pio_state_machines[MAX_PHOTODIODES] = {0, 1, 2, 3};
// buffer for received stream for decoding
uint32_t rx_buffer[2] = { 0 };
// stream for lookup
uint32_t lookup_stream[2] = { 0 };
// beam for lookup (constructed out of lookup stream)
uint64_t beam = 0;
// lfsr index from lookup
uint32_t lfsr_index;
// lfsr from lookup
uint8_t lfsr;
// represents index of station where beam was emitted
uint8_t beam_station_index = 0;
// counter for fresh beams that are received
int count_dirty = 0;
// counter of successful decodes
uint32_t count = 0;
// individual counter of successful decodes
uint32_t individual_count[4] = { 0 };
// timestamp for beam reception
unsigned long count_timestamp = 0;
// difference of saved and received beam
uint32_t beam_difference = 0;

// - POSITIONING ----------------------------------------------------------------------------------
// timestamp for position calculation
unsigned long last_timestamp;
// data representing the decoded beams recevied by the sensors
SensorData sensor_data[NO_PHOTODIODES];
// position of the photodiodes
SensorPosition sensor_position[NO_PHOTODIODES];
// position of the middle of the tile
vec3d tile_position = {0};
// vector that points from middle to front side of the tile (from usb connector to the point between d4&d1)
vec3d forward_vector = {0};
// vector that points 90 degrees to the plane which is constructed by the 4 photodiodes
vec3d up_vector = {0};
// vector that points from middle to the left side of the tile
vec3d side_vector = {0};

// - CONFIGURATION --------------------------------------------------------------------------------
// flag for storing config permanent in file
uint8_t rewrite_file = 0;
// timestamp for storing file
unsigned long file_timestamp = 0;
// station positions and heading
// stations at origin, looking to x direction when no config file present
vec3d station_positions[MAX_BASE_STATIONS] = { {0,0,0}, { 0, 0, 0 } };
mat3x3 station_orientations[MAX_BASE_STATIONS] = { { 1, 0, 0, 0, 1, 0, 0, 0, 1 }, { 1, 0, 0, 0, 1, 0, 0, 0, 1 } };
// station index of the received config
uint8_t config_station_index = 0;
// flag that quick configuration data for the station has already been sent
uint8_t config_done[MAX_BASE_STATIONS] = {0};
// last received command of normal config
uint8_t last_received_command_number = 0;
// normal configuration step
enum ConfigurationSteps config_step = wait_next_position;
// if current config mode is quick config, used for length of sent data packet via i2c
uint8_t quick_config_mode = 0;

// - SERIAL ---------------------------------------------------------------------------------------
// serial incomming character (c - config)
char incoming_serial = 0;

// - CONTROL --------------------------------------------------------------------------------------
// operation mode of tile
enum TileOperationModes operation_mode = normal;

// normal mode: 5 for max range
// calib mode: 10-30 for best reception (about half a meter away!)
int skipped_bits = 10;

// - SETUP ----------------------------------------------------------------------------------------
void setup()
{
  // start serial connection to pc
  Serial.begin(115200);

  // pin for i2c int
  pinMode(I2C_INT_TO_ESP, OUTPUT);
  digitalWrite(I2C_INT_TO_ESP, LOW);

  // configure i2c 
  // use Wire1, Wire does not work for rp2040! -> always does NACK
  Wire1.setClock(I2C_FREQUENCY);
  Wire1.setSDA(I2C_SDA);
  Wire1.setSCL(I2C_SCL);
  // join i2c bus with address
  Wire1.begin(i2c_address); 
  // register i2c callbacks
  Wire1.onReceive(receive_i2c_data);
  Wire1.onRequest(send_i2c_data);


  // blink front leds one time to signal setup
  for (int i = 0; i < NUM_LEDS; i++) 
  {
    pixels.setPixelColor(i, COL_BLUE);
  }
  // perform update on leds
  pixels.show();
  delay(LED_DELAY_TIME);
  // turn front leds off
  turn_lights_off();

  #if (DBG_SERIAL == 1)
    // wait for serial connection
    while (!Serial) {}
    Serial.println("[DBG_SERIAL] Serial Port Connected");
  #endif

  // ensure file is present at beginning
  init_load_config_file();
  

  #if (SHOW_LOADED_CONFIG_DATA == 1)
    // print the loaded values in the positioning structs
    for(int i = 0; i < MAX_BASE_STATIONS; ++i)
    {
      Serial.print("Base Station ");
      Serial.println(i+1);
      Serial.println("Position");
      print_vector(station_positions[i]);
      Serial.println("Heading");
      print_mat3x3(station_orientations[i]);
    }
  #endif
  
  // builtin led is used to signal configuration done
  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(LED_BUILTIN, HIGH);
  
  // start sensors in receiving mode
  configure_sensors();

  // photodiodes configured
  digitalWrite(LED_BUILTIN, LOW);

  // ------------------------------------------------------------------------------------------------------------------------
  // setup pio
  setup_pio();

  // pio initialized
  for(int i = 0; i < 3; ++i)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(300);
    digitalWrite(LED_BUILTIN, LOW);
    delay(300);
  }
  
  // ------------------------------------------------------------------------------------------------------------------------
  // attach interrupts for e-pins
  for(int i = 0; i < NO_PHOTODIODES; ++i)
  {
    attachInterrupt(digitalPinToInterrupt(photodiode_e_pin[i]), isrs[i], FALLING);
  }

  // set one blue LED to show setup complete
  // turn on LED to signal data received
  pixels.setPixelColor(0, COL_BLUE);
  // perform update on leds
  pixels.show();
  last_timestamp = millis();
}



// ======================================================================================================================================================================
// === MAIN PROGRAM LOOP ===
// ======================================================================================================================================================================
void loop()
{
  // ------------------------------------------------------------------------------------------------------------------------
  // GET BEAMS FROM PIO AND DECODE
  // check if e pin got low, and data is available
  for(int i = 0; i < NO_PHOTODIODES; ++i)
  {
    if(e_flag[i] == 1)
    {
      // fifo is combined -> (8*4 bytes can be recorded) -> 8 elements in fifo buffer (fifo_level)
      // first 8 bytes should already be decent enough for us, and also improves range
      // this would mean fifo level 2 and reading of 2 elements

      // check for enough data in fifo from pio state machine
      if(pio_sm_get_rx_fifo_level(pio, pio_state_machines[i]) >= 2)
      {
        
        // get data from fifo
        for(int j = 0; j < 2; ++j)
        {
          rx_buffer[j] = pio_sm_get_blocking(pio, pio_state_machines[i]);
          lookup_stream[j] = reverse_bits(rx_buffer[j]);
        }

        // clear old data from fifo
        pio_sm_clear_fifos(pio, pio_state_machines[i]);
        
        // reset interrupt flag
        e_flag[i] = 0;

		    // addapted pio version only receives 44 bits
        // MSB is the highest bit -> first 10 bits are skipped as decoding starts with LSB
        beam = 0;
        beam += (((uint64_t)lookup_stream[0]) << 22);
        beam += lookup_stream[1];

        // decode received beam
        if(get_unique_index(beam, &lfsr_index, &lfsr))
        {
          // each station has 2 lfsrs
          beam_station_index = lfsr/2;

          // check if beam belongs to current index, otherwise write index to other number
          if(sensor_data[i].beams[beam_station_index].is_beam_1)
          {
            // calculate difference to current beam
            if(sensor_data[i].beams[beam_station_index].index_1 < lfsr_index)
            {
              beam_difference = lfsr_index - sensor_data[i].beams[beam_station_index].index_1;
            }
            else
            {
              beam_difference = sensor_data[i].beams[beam_station_index].index_1 - lfsr_index;
            }
            if(beam_difference < BEAM_INDEX_DIFFERENCE)
            {
              // same beam, safe in the same index field
              sensor_data[i].beams[beam_station_index].dirty_1 = 1;
              sensor_data[i].beams[beam_station_index].index_1 = lfsr_index;
            }
            else
            {
              sensor_data[i].beams[beam_station_index].dirty_2 = 1;
              sensor_data[i].beams[beam_station_index].index_2 = lfsr_index;
            }
          }
          else
          {
            // calculate difference to current beam
            if(sensor_data[i].beams[beam_station_index].index_2 < lfsr_index)
            {
              beam_difference = lfsr_index - sensor_data[i].beams[beam_station_index].index_2;
            }
            else
            {
              beam_difference = sensor_data[i].beams[beam_station_index].index_2 - lfsr_index;
            }
            if(beam_difference < BEAM_INDEX_DIFFERENCE)
            {
              // same beam, safe in the same index field
              sensor_data[i].beams[beam_station_index].dirty_2 = 1;
              sensor_data[i].beams[beam_station_index].index_2 = lfsr_index;
            }
            else
            {
              sensor_data[i].beams[beam_station_index].dirty_1 = 1;
              sensor_data[i].beams[beam_station_index].index_1 = lfsr_index;
            }
          }
          // change comparison to next index, so if 
          sensor_data[i].beams[beam_station_index].is_beam_1 = !(sensor_data[i].beams[beam_station_index].is_beam_1);

          count += 1;
          //individual_count[i]++;
        }
      }
    }
  }
  // ------------------------------------------------------------------------------------------------------------------------
  #if (USE_SERIAL == 1)
    if(operation_mode == normal)
    {
      if(Serial.available() > 0)
      {
        incoming_serial = Serial.read();
        
        if(incoming_serial == 'c')
        {
          // switch to serial config mode
          operation_mode = config_serial;
          // turn on four blue leds to signal serial config
          pixels.setPixelColor(0, COL_BLUE);
          pixels.setPixelColor(2, COL_BLUE);
          pixels.setPixelColor(4, COL_BLUE);
          pixels.setPixelColor(6, COL_BLUE);
          pixels.show();
        }
        if(incoming_serial == 'q')
        {
          // switch to quick serial config mode
          operation_mode = quick_config_serial;
          // reset flags
          for(uint8_t bs_index = 0; bs_index < MAX_BASE_STATIONS; ++bs_index)
          {
            config_done[bs_index] = 0;
          }

          // reset dirty bits to get fresh values
          reset_dirty_bits();

          // turn on two blue leds to signal serial quick config
          pixels.setPixelColor(0, COL_BLUE);
          pixels.setPixelColor(6, COL_BLUE);
          pixels.show();
        }
        if(Serial.available() > 0)
        {
          // read '\n' to clear buffer
          incoming_serial = Serial.read();
        }
      }
    }
  #endif
  // ------------------------------------------------------------------------------------------------------------------------
  // CALCULATION OF POSITION AND HEADING

  /*// for logging of correctly recorded beams
  if((millis() - count_timestamp) > 1000)
  {
    for(int i = 0; i < 4; ++i)
    {
      Serial.print(individual_count[i]);
      Serial.print(" ");
      individual_count[i] = 0;
    }
    Serial.println(count);
    count = 0;
    count_timestamp = millis();
  }*/
  if((millis() - last_timestamp) > 20)
  {
    // check for i2c
    if(i2c_received == 1)
    {
      i2c_received = 0;
      #if (DBG_I2C == 1)
        Serial.print("[DBG_I2C] data received with command: ");
        Serial.println((char)i2c_receive_buffer[0]);
      #endif
      // evaluate data, as it was last part of packet and we should have time until next comunication
      if(i2c_receive_buffer[0] == 'c')
      {
        // switch to i2c config mode
        quick_config_mode = 0;
        operation_mode = config_i2c;
        // turn on four yellow leds to signal i2c config
        pixels.setPixelColor(0, COL_YELLOW);
        pixels.setPixelColor(2, COL_YELLOW);
        pixels.setPixelColor(4, COL_YELLOW);
        pixels.setPixelColor(6, COL_YELLOW);
        pixels.show();
      }
      else if(i2c_receive_buffer[0] == '1')
      {
        #if (DBG_CONFIG == 1)
          Serial.println("[DBG_CONFIG] 1 received");
        #endif
        start_normal_config_record();
        // set light
        last_received_command_number = 1;
        pixels.setPixelColor(5, COL_BLUE);
        pixels.setPixelColor(7, COL_BLUE);
        pixels.show();
      }
      else if(i2c_receive_buffer[0] == '2')
      {
        #if (DBG_CONFIG == 1)
          Serial.println("[DBG_CONFIG] 2 received");
        #endif
        start_normal_config_record();
        last_received_command_number = 2;
        pixels.setPixelColor(1, COL_BLUE);
        pixels.show();
      }
      else if(i2c_receive_buffer[0] == '3')
      {
        start_normal_config_record();
        last_received_command_number = 3;
        pixels.setPixelColor(3, COL_BLUE);
        pixels.show();
      }
      else if(i2c_receive_buffer[0] == '4')
      {
        #if (DBG_CONFIG == 1)
          Serial.println("[DBG_CONFIG] 4 received");
        #endif
        start_normal_config_record();
        last_received_command_number = 4;
        pixels.setPixelColor(5, COL_BLUE);
        pixels.show();
      }
      else if(i2c_receive_buffer[0] == '5')
      {
        #if (DBG_CONFIG == 1)
          Serial.println("[DBG_CONFIG] Waiting for stop command or configuration data");
        #endif
        // set tile to wait for config data
        config_step = wait_config_data;
        // turn last lights to green
        pixels.setPixelColor(5, COL_GREEN);
        pixels.show();
      }
      else if(i2c_receive_buffer[0] == 'q')
      {
        // switch to i2c quick config mode
        quick_config_mode = 1;
        operation_mode = quick_config_i2c;
        // reset flags
        for(uint8_t bs_index = 0; bs_index < MAX_BASE_STATIONS; ++bs_index)
        {
          config_done[bs_index] = 0;
        }

        // reset dirty bits to get fresh values
        reset_dirty_bits();

        // turn on two blue leds to signal i2c quick config
        pixels.setPixelColor(0, COL_YELLOW);
        pixels.setPixelColor(6, COL_YELLOW);
        pixels.show();
      }
      else if(i2c_receive_buffer[0] == 'd')
      {
        // it's config data, parse it
        config_station_index = i2c_receive_buffer[1];
        memcpy(&station_positions[config_station_index], &i2c_receive_buffer[2], 12);
        memcpy(&station_orientations[config_station_index], &i2c_receive_buffer[14], 36);

        // set flags to rewrite config file
        rewrite_file = 1;
        file_timestamp = millis();
        // set led to signal config received
        pixels.setPixelColor(5, COL_GREEN);
        pixels.show();
        
        #if (DBG_CONFIG == 1)
          Serial.println("[DBG_CONFIG] got new i2c configuration");
          Serial.println("station index");
          Serial.println(config_station_index);
          Serial.println("station position");
          print_vector(station_positions[config_station_index]);
          Serial.println("station orientation");
          print_mat3x3(station_orientations[config_station_index]);
        #endif
      }
      else if(i2c_receive_buffer[0] == 's')
      {
        Serial.println("stop");
        #if (DBG_CONFIG == 1)
          Serial.println("[DBG_CONFIG] got stop signal");
        #endif
        // it's stop command return to normal operation
        operation_mode = normal;
        // reset config steps for normal configuration
        config_step = wait_next_position;
        // turn of config lights
        turn_lights_off();
      }
    }

    // store config file with new data
    if(rewrite_file == 1)
    {
      if(millis() - file_timestamp > CONFIG_FILE_DELAY)
      {
        // reset flag
        rewrite_file = 0;

        // store new config file
        store_config_file();
      }
    }

// ==============================================================================================
// ================== operation mode ======================================
// ==============================================================================================

    switch(operation_mode)
    {
      case normal:
        perform_positioning();
        break;
      case config_serial:
        perform_config_serial();
        break;
      case quick_config_serial:
        perform_quick_config_serial();
        break;
      case config_i2c:
        perform_config_i2c();
        break;
      case quick_config_i2c:
        perform_quick_config_i2c();
        break;
    }

    last_timestamp = millis();
  }
}

