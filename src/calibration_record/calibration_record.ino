/*
 * L-Cube Calibration Record
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

/*
---------------------------------------------------------------------
--- rp2040 program to get calibration of the base stations
--- -> flash to tile which is placed about 1-1.5m away from a base station
--- -> calibration is the printed to serial out
--- -> copy this calibration into the rigth index of the array from bs_calibration_data.h file
--- 
*/

#include <hardware/pio.h>
#include "differential_manchester.pio.h"

#include "lookup.h"
#include "sweeping.h"
#include "isr.h"
#include "sensor_config.h"
#include "config.h"
#include "mmath.h"
#include "positioning.h"
#include "bs_calibration.h"
#include <Adafruit_NeoPixel.h>

// class for ws2812 leds on top (neopixels)
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_LEDS, PIN_LEDS, NEO_GRB + NEO_KHZ800);

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

// 20 ms timestamp
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
// timestamp for position calculation
unsigned long last_timestamp_calibration;
// current bit for calibration
uint8_t calibration_bit = 0;


// contains amount of received bits in one second (should be 100, otherwise beam reception is bad)
uint8_t call_counter = 0;

// stations at origin, looking to x direction when no config file present
vec3d station_positions[MAX_BASE_STATIONS] = { {0, 0, 0}, { 0, 0, 0 } };
mat3x3 station_orientations[MAX_BASE_STATIONS] = { { 1, 0, 0, 0, 1, 0, 0, 0, 1 }, { 1, 0, 0, 0, 1, 0, 0, 0, 1 } };

// - CONTROL --------------------------------------------------------------------------------------
// operation mode of tile
enum TileOperationModes operation_mode = normal;

// calib mode: 10-30 for best reception (about half a meter away!)
int skipped_bits = 10;

// turns off all front lights
void turn_lights_off()
{
  for (int i = 0; i < NUM_LEDS; i++) 
  {
    pixels.setPixelColor(i, COL_OFF);
  }
  pixels.show();
}


// - SETUP ----------------------------------------------------------------------------------------
void setup()
{
  // start serial connection to pc
  Serial.begin(115200);

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
      // first 8 bytes are enough for us
      // this would mean fifo level 2 and reading of 2 elements

      // check for enough data in fifo from pio state machine
      if(pio_sm_get_rx_fifo_level(pio, pio_state_machines[i]) >= 2)
      {
        
        for(int j = 0; j < 2; ++j)
        {
          rx_buffer[j] = pio_sm_get_blocking(pio, pio_state_machines[i]);
          lookup_stream[j] = reverse_bits(rx_buffer[j]);
        }
        
        pio_sm_clear_fifos(pio, pio_state_machines[i]);
        
        e_flag[i] = 0;

        // get and prepare beam
        beam = 0;
        beam += (((uint64_t)lookup_stream[0]) << 32);
        beam += lookup_stream[1];

        beam = beam >> (30 - skipped_bits);

        // decode received beam
        if(get_unique_index(beam, &lfsr_index, &lfsr))
        {
          beam_station_index = lfsr/2;

          if(sensor_data[i].beams[beam_station_index].is_beam_1)
          {
            sensor_data[i].beams[beam_station_index].dirty_1 = 1;
            sensor_data[i].beams[beam_station_index].index_1 = lfsr_index;
          }
          else
          {
            sensor_data[i].beams[beam_station_index].dirty_2 = 1;
            sensor_data[i].beams[beam_station_index].index_2 = lfsr_index;
          }
          sensor_data[i].beams[beam_station_index].is_beam_1 = !(sensor_data[i].beams[beam_station_index].is_beam_1);

          calibration_bit = lfsr & 0x01;
          
          // get calibration
          perform_calibration(calibration_bit, beam_station_index, i, lfsr_index);

          count += 1;
          individual_count[i] += 1;
        }
      }
    }
  }

  
  #if (DBG_CALIB == 1)
    if((millis() - last_timestamp_calibration) > 1000)
    {
      if((call_counter < 100) || (call_counter > 101))
      {
        Serial.println("[DBG_CALIB] Beam reception might be to bad for recording calibration!");
        Serial.print("[DBG_CALIB] bitcount: ");
        Serial.println(call_counter);
      }
      call_counter = 0;
      last_timestamp_calibration = millis();
    }
  #endif


// ==============================================================================================
// ================== operation mode ======================================
// ==============================================================================================

/*
  // TEST CALIBRATION WITH STATION 1
  if(millis() - last_timestamp > 2000)
  {

    perform_positioning();

    last_timestamp = millis();
  }
/**/
}


