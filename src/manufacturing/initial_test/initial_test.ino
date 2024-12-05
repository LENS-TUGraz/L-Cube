/*
 * L-Cube Initial Test
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

// ============================================================================
// === INITIAL TEST PROGRAMM USED TO VERIFY FUNCTIONALITY OF HARDWARE !!!!! ===
// ============================================================================

// HOW TO USE:
// Place the receiving tile about one meter away from a base station which is configured with index 0

// HOW THE HARDWARE SHOULD BEHAVE:
// First, the LED on the back side should turn on, then the LEDs on the front side should turn on in blue light one by one.
// After all LEDs are lighting, they should turn off again (front and back).
// The back LED now turn on again and lights until the setup() is completed.
// Within the setup, three LEDs should get turned on in green light one by one.
// In loop(), the remaining LEDS should turn on in green light one by one.
// If setup is completed and correct, all LEDs should blink in green and white.

// WHAT EVERY LED INDICATES: (RED IF TEST FAILED, GREEN IF SUCCESSFUL)
// LED1: INDICATES LED TEST
// LED2: INDICATES IF PIO IS SETUP CORRECTLY
// LED3: INDICATES IF ALL INTERRUPTS CAN BE REGISTERED
// LED4: INDICATES IF AT LEAST 80% OF THE AVAILABLE BEAMS ARE RECEIVED
// LED5: INDICATES IF RECEIVED ANGLES ARE WITHIN SPECIFIED LIMITS (in the middle of the viewing area of the base station)
// LED6: INDICATES IF THE SWEEPING ALGORITHM RETURNS A CORRECT DISTANCE WITHIN THE SPECIFIED LIMIT (hw sould be placed about 1m away from base station)
// LED7/8: LAST TWO LEDS IN WHITE INDICATE THAT TESTS ARE COMPLETED -> HARDWARE WORKS IF ALL OTHER LEDS ARE GREEN
// IF THE PROGRAM TAKES MORE THAN 15 SECONDS BEFORE NEXT LED GETS TURNED ON, IT PROSSIBLY THREW A PANIC()

// SENSOR 0 IS DIODE 4 ON HARDWARE (REST IST THE SAME INDEX)

#include <hardware/pio.h>
#include "differential_manchester.pio.h"

#include "lookup.h"
#include "sweeping.h"
#include "isr.h"
#include "sensor_config.h"
#include "config.h"
#include "mmath.h"
#include <Adafruit_NeoPixel.h>

// class for ws2812 leds on top
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_LEDS, PIN_LEDS, NEO_GRB + NEO_KHZ800);

// flags to set in ISR and handle in loop()
uint8_t e_flag[MAX_PHOTODIODES] = { 0 };
// isr declarations
void (*isrs[MAX_PHOTODIODES])() = { isr_e0, isr_e1, isr_e2, isr_e3};

// data and envelope pins of the receiving sensors (photodiode+ts4231)
uint8_t photodiode_d_pin[MAX_PHOTODIODES] = {D_PINS};
uint8_t photodiode_e_pin[MAX_PHOTODIODES] = {E_PINS};

// rp2040 contains two pio blocks (pio0,pio1) with four state machines eachRaspberry Pi Pico enthält zwei PIO-Blöcke (pio0 und pio1) mit je vier Zustandsmaschinen, die sich einen Speicher von 32 Instruktionen teilen. // adafruit uses pi0 sm 0
// we only use pio1 entirely -> pio0,sm0 is used by adafruit neopixel
PIO pio = pio1;
uint pio_state_machines[4] = {0, 1, 2, 3};

// struct for a received beam
typedef struct _BeamParameter {
  uint8_t dirty_1;
  uint32_t index_1;
  uint8_t dirty_2;
  uint32_t index_2;
  uint8_t is_beam_1;
} BeamParameter;

typedef struct _SensorData {
  // beams can be received from up to 16 stations
  BeamParameter beams[MAX_BASE_STATIONS];
} SensorData;

SensorData sensor_data[NO_PHOTODIODES];

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
uint8_t beam_index = 0;

// station positions and heading
// stations at origin, looking to x direction
vec3d station_positions[MAX_BASE_STATIONS] = { { 0, 0, 0 }, { 0, 0, 0 } };
mat3x3 station_orientations[MAX_BASE_STATIONS] = { { 1, 0, 0, 0, 1, 0, 0, 0, 1 }, { 1, 0, 0, 0, 1, 0, 0, 0, 1 } };


// --------------------------------------------------------------
// tests general
// represents the test state in loop()
uint8_t test_state = 0;
// get set to 1 if a test is passed to print final test results in a list
uint8_t test_results[NUM_LEDS] = {0};
// timestamps for time measurements
unsigned long last_timestamp;
// --------------------------------------------------------------
// decoding test
// current counter of decoding cycles
uint8_t current_decoding_cycle = 0;
// counter of successful decodes in a second for each sensor individually
uint32_t count = 0;
// counter of successful decodes in a second
uint32_t individual_count[4] = { 0 };
// array of counters for evaluation
uint32_t decoded_count[DECODING_CYCLES] = {0};
uint32_t decoded_individual_count[DECODING_CYCLES][NO_PHOTODIODES] = {0};
// --------------------------------------------------------------
// azimuth elevation calculation test
// current number of calculations performed
uint8_t current_calculation_cycle[NO_PHOTODIODES] = {0};
// indicates if a sensor has passed calculation test
uint8_t calculation_passed[NO_PHOTODIODES] = {0};
// counter of receiving cycles for timeout
uint8_t current_azimuth_elevation_cycle = 0;
// --------------------------------------------------------------
// sweeping test
// receiving cycle counter for timeout
uint8_t current_sweeping_cycle = 0;
// counter for new beams for sweeping
int count_dirty = 0;
// contains the distance to the base station of the sweeping calculation
float sweeping_result = 0;



// LH2 sends msb first, the pio receiver does it exactly the other way
// (MSB is in last bit therefore reverse the bits)
uint32_t reverse_bits(uint32_t in) {
  uint32_t reversed = 0;
  for (int i = 0; i < 32; ++i) {
    reversed |= ((in & 0x00000001) << (31 - i));
    in = (in >> 1);
  }
  return reversed;
}

void finish_tests()
{
  // set last two leds to white to indicate test end
  pixels.setPixelColor(2, COL_WHITE);
  pixels.setPixelColor(3, COL_WHITE);
  pixels.show();

  // print test results
  Serial.println("=============================================================");
  Serial.println("=============================================================");
  Serial.println("=== TEST RESULTS ===");

  Serial.print("LED_TEST:          ");
  if(test_results[0] == 1){
    Serial.println("PASSED");
  } else {
    Serial.println("FAILED - NOT ALL LEDS WORKING");
  }

  Serial.print("PIO_SETUP:         ");
  if(test_results[1] == 1){
    Serial.println("PASSED");
  } else {
    Serial.println("FAILED - PIO FUNCIONALITY INCORRECT");
  }

  Serial.print("INTERRUPTS:        ");
  if(test_results[2] == 1){
    Serial.println("PASSED");
  } else {
    Serial.println("FAILED - INTERRUPTS COULD NOT BE REGISTERED");
  }

  Serial.print("DECODING_BEAMS :   ");
  if(test_results[3] == 1){
    Serial.println("PASSED");
  } else {
    Serial.println("FAILED - MOST OF THE RECEIVED BEAMS WERE NOT DECODED");
  }

  Serial.print("AZIMUTH_ELEVATION: ");
  if(test_results[4] == 1){
    Serial.println("PASSED");
  } else {
    Serial.println("FAILED - BOARD RECEIVES ANGLES THAT ARE NOT IN THE MIDDLE OF THE BASE STATION'S VIEWING ANGLE");
  }

  Serial.print("SWEEPING_TEST:     ");
  if(test_results[5] == 1){
    Serial.println("PASSED");
  } else {
    Serial.println("FAILED - RESULTING SWEEPING DISTANCE IS OUT OF THE SPECIFIED LIMITS");
  }


  Serial.println("=============================================================");
  Serial.println("=============================================================");

  // Do nothing
  while(1)
  {
    delay(DELAY_TIME);
  }
}


void setup() {
  // start serial connection to pc
  Serial.begin(115200);

  #if (DBG_SERIAL == 1)
    // wait for serial connection
    while (!Serial) {}
  #endif

  Serial.println("===========================================================================");
  Serial.println("[INIT_TEST] Serial port connected");

  // ------------------------------------------------------------------------------------------------------------------------
  Serial.println("===========================================================================");
  Serial.println("[INIT_TEST] Starting LED test");

  Serial.println("[LED_TEST] Turning on user LED");
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(DELAY_TIME);

  Serial.println("[LED_TEST] Turning on front LEDs");
  pixels.begin();

  // set 1 by 1 pixel to blue (in hardware the leds are connected counterclockwise, thats why we have to set the leds in this weird way)
  for (int i = 0; i < NUM_LEDS; i++) 
  {
    if(i < 2)
    {
      pixels.setPixelColor(i, COL_BLUE);
    }
    else if(i < 4)
    {
      pixels.setPixelColor(i+4, COL_BLUE);
    }
    else if(i < 6)
    {
      pixels.setPixelColor(i, COL_BLUE);
    }
    else
    {
      pixels.setPixelColor(i-4, COL_BLUE);
    }
    
    // perform update on leds
    pixels.show();
    delay(DELAY_TIME);
  }

  Serial.println("[LED_TEST] All LEDs should all light in green now");
  delay(DELAY_TIME);

  // turn off user led
  digitalWrite(LED_BUILTIN, LOW);
  // turn off front leds
  for (int i = 0; i < NUM_LEDS; i++) 
  {
    pixels.setPixelColor(i, COL_OFF);
  }
  pixels.show();

  Serial.println("[LED_TEST] All LEDs should be turned off");
  delay(DELAY_TIME);

  // ------------------------------------------------------------------------------------------------------------------------
  Serial.println("===========================================================================");
  Serial.println("[INIT_TEST] Starting photodiode configuration");

  pinMode(PIN_USERBUTTON, INPUT);

  // builtin led is used to signal configuration done
  digitalWrite(LED_BUILTIN, HIGH);

  // only returns if all photodiodes configured
  configure_sensors();
  

  Serial.println("[PHOTODIODE_CONFIGURATION] All photodiodes configured");
  // turn on first led
  pixels.setPixelColor(0, COL_GREEN);
  pixels.show();
  test_results[0] = 1;
  delay(DELAY_TIME);

  // ------------------------------------------------------------------------------------------------------------------------
  Serial.println("===========================================================================");
  Serial.println("[INIT_TEST] Starting setting up pio");
  // setup pio
  setup_pio();

  // turn on second led
  pixels.setPixelColor(1, COL_GREEN);
  pixels.show();
  test_results[1] = 1;
  delay(DELAY_TIME);
  Serial.println("[PIO_SETUP] PIO ready to run");

  // ------------------------------------------------------------------------------------------------------------------------
  // attaching interrupts for envelope-pins
  Serial.println("===========================================================================");
  Serial.println("[INIT_TEST] Starting attaching interrupts");
  for (int i = 0; i < NO_PHOTODIODES; ++i) 
  {
    attachInterrupt(digitalPinToInterrupt(photodiode_e_pin[i]), isrs[i], FALLING);
  }

  Serial.println("[INTERRUPTS] All interrupts attached");
  // turn on third led
  pixels.setPixelColor(6, COL_GREEN);
  pixels.show();
  test_results[2] = 1;
  delay(DELAY_TIME);

  // ------------------------------------------------------------------------------------------------------------------------
  Serial.println("===========================================================================");
  Serial.println("[INIT_TEST] setup() completed, starting with loop()");
  digitalWrite(LED_BUILTIN, LOW);
  delay(DELAY_TIME);
  delay(DELAY_TIME);
  // first timestamp for main loop
  last_timestamp = millis();
}

// ======================================================================================================================================================================
// === MAIN PROGRAM LOOP ===
// ======================================================================================================================================================================
void loop() {
  // ------------------------------------------------------------------------------------------------------------------------
  // RECEIVING AND DECODING BEAMS PART
  // check if e pin got low, and data is available
  for (int i = 0; i < NO_PHOTODIODES; ++i) 
  {
    if (e_flag[i] == 1) 
    {
      // fifo is combined -> (8*4 bytes can be recorded) -> 8 elements in fifo buffer (fifo_level)
      // first 8 bytes should already be decent enough for us, and also improves range
      // this would mean fifo level 2 and reading of 2 elements

      // check for enough data in fifo from pio state machine
      if (pio_sm_get_rx_fifo_level(pio, pio_state_machines[i]) >= 2) 
      {
        for (int j = 0; j < 2; ++j) 
        {
          rx_buffer[j] = pio_sm_get_blocking(pio, pio_state_machines[i]);
          lookup_stream[j] = reverse_bits(rx_buffer[j]);
        }

        pio_sm_clear_fifos(pio, pio_state_machines[i]);

        e_flag[i] = 0;

        // decoding beam
        beam = 0;
        beam += (((uint64_t)lookup_stream[0]) << 32);
        beam += lookup_stream[1];

        /*beam = 0;
          beam += (((uint64_t)lookup_stream[0]) << 2);
          beam += (((uint64_t)lookup_stream[1]) >> 30);
          /**/

        // decode received beam
        if (get_unique_index(beam, &lfsr_index, &lfsr)) 
        {
          beam_index = lfsr / 2;

          if (sensor_data[i].beams[beam_index].is_beam_1) 
          {
            sensor_data[i].beams[beam_index].dirty_1 = 1;
            sensor_data[i].beams[beam_index].index_1 = lfsr_index;
          } else 
          {
            sensor_data[i].beams[beam_index].dirty_2 = 1;
            sensor_data[i].beams[beam_index].index_2 = lfsr_index;
          }
          sensor_data[i].beams[beam_index].is_beam_1 = !(sensor_data[i].beams[beam_index].is_beam_1);

          count ++;
          individual_count[i]++;
        }
      }
    }
  }

  // ------------------------------------------------------------------------------------------------------------------------
  // FURTHER PART OF INITIAL TEST:
  switch (test_state)
  {
    // ---------------------------------------------------------------------------------------------------------------------
    // decoding test
    case 0:
      Serial.println("===========================================================================");
      Serial.println("[INIT_TEST] Starting decoding beam test");
      // reset counters
      count = 0;
      for(int i = 0; i < NO_PHOTODIODES; ++i)
      {
        individual_count[i] = 0;
      }
      test_state  = 1;
      last_timestamp = millis();
      break;
    
    case 1:
      if ((millis() - last_timestamp) > 1000) 
      {
        // recording data
        decoded_count[current_decoding_cycle] = count;
        count = 0;
        for(int i = 0; i < NO_PHOTODIODES; ++i)
        {
          decoded_individual_count[current_decoding_cycle][i] = individual_count[i];
          individual_count[i] = 0;
        }
        current_decoding_cycle++;
        
        if(current_decoding_cycle == DECODING_CYCLES)
        {
          Serial.println("[DECODING_BEAMS] Testresults for decoding beams:");
          for(int i = 0; i < DECODING_CYCLES; ++i)
          {
            Serial.println("[DECODING_BEAMS] -----------------------------------------------------------");
            Serial.print("[DECODING_BEAMS] Cycle ");
            Serial.print(i);
            Serial.print(": total = ");
            Serial.println(decoded_count[i]);
            Serial.print("[DECODING_BEAMS] Individual: ");
            for(int j = 0; j < NO_PHOTODIODES; ++j)
            {
              Serial.print(decoded_individual_count[i][j]);
              Serial.print(" ");
            }
            Serial.println("");
          }

          float beam_decoding_percentage = 0;
          // check test results with code:
          for(int i = 0; i < DECODING_CYCLES; ++i)
          {
            beam_decoding_percentage += decoded_count[i];
          }
          beam_decoding_percentage /= (DECODING_CYCLES* 100.0  * NO_PHOTODIODES);
          // in percent
          beam_decoding_percentage *= 100;
          if(beam_decoding_percentage > 80)
          {
            pixels.setPixelColor(7, COL_GREEN);
            pixels.show();
            test_results[3] = 1;
            Serial.print("[DECODING_BEAMS] PASSED WITH ");
          }
          else
          {
            pixels.setPixelColor(7, COL_RED);
            pixels.show();
            Serial.print("[DECODING_BEAMS] FAILED WITH ");
          }
          
          Serial.print(beam_decoding_percentage);
          Serial.println("\%");
          test_state = 2;
        }

        last_timestamp = millis();
      }
      break;

    // ---------------------------------------------------------------------------------------------------------------------
    // azimuth elevation test
    case 2:
      Serial.println("===========================================================================");
      Serial.println("[INIT_TEST] Starting azimuth elevation calculation test");
      current_azimuth_elevation_cycle = 0;
      test_state = 3;
      last_timestamp = millis();
      break;

    case 3:
      if ((millis() - last_timestamp) > 20)
      {
        float azimuth = 0;
        float elevation = 0;
        uint8_t passed_counter = 0;

        // receive a correct beam within 20 cycles (test also fails if 10 beams out of specified area)
        current_azimuth_elevation_cycle++;
        if(current_azimuth_elevation_cycle == TIMEOUT_CYCLES_AZIMUTH_ELEVATION)
        {
          Serial.println("[AZIMUTH_ELEVATION] FAILED WITH TIMEOUT");
          pixels.setPixelColor(4, COL_RED);
          pixels.show();
          test_state = 4;
          break;
        }
        
        for (int i = 0; i < NO_PHOTODIODES; ++i) 
        {          
          if(calculation_passed[i] == 1)
          {
            passed_counter++;
          }
          else
          {
            if (calculate_azimuth_elevation(0, sensor_data[i].beams[0].index_1, sensor_data[i].beams[0].index_2, &azimuth, &elevation)) 
            {
              current_calculation_cycle[i]++;
              
              // we want a correct calculation with below 20 degrees away from the heading of the base station with index 1
              Serial.println("[AZIMUTH_ELEVATION] Calculation result: Sensor ");
              Serial.print(i);
              Serial.print(" Round ");
              Serial.print(current_calculation_cycle[i]);
              Serial.print(" azimuth: ");
              Serial.print(to_degree(azimuth));
              Serial.print(" elevation: ");
              Serial.println(to_degree(elevation));

              if((azimuth < AZIMUTH_ELEVATION_LIMIT) && (elevation < AZIMUTH_ELEVATION_LIMIT))
              {
                calculation_passed[i] = 1;
              }
            }
            if(current_calculation_cycle[i] == AZIMUTH_ELEVATION_MAX_TRIES)
            {
              Serial.print("[AZIMUTH_ELEVATION] FAILED FOR SENSOR ");
              Serial.println(i);
              pixels.setPixelColor(4, COL_RED);
              pixels.show();
              test_state = 4;
              break;
            }
          }
          if(passed_counter == NO_PHOTODIODES)
          {
            Serial.println("[AZIMUTH_ELEVATION] PASSED");
            test_results[4] = 1;
            pixels.setPixelColor(4, COL_GREEN);
            pixels.show();
            test_state = 4;
            break;
          }
        }
        last_timestamp = millis();
      }
      break;
    // ---------------------------------------------------------------------------------------------------------------------
    // sweeping test
    case 4:
      Serial.println("===========================================================================");
      Serial.println("[INIT_TEST] Starting sweeping distance test");
      test_state = 5;
      last_timestamp = millis();
      break;

    case 5:
      // we use base station with index 1 only
      if ((millis() - last_timestamp) > 20)
      {
        current_sweeping_cycle++;
        if(current_sweeping_cycle == TIMEOUT_CYCLES_SWEEPING)
        {
          Serial.println("[SWEEPING_TEST] FAILED WITH TIMEOUT");
          pixels.setPixelColor(5, COL_RED);
          pixels.show();
          finish_tests();
        }

        count_dirty = 0;
        for (int i = 0; i < NO_PHOTODIODES; ++i) {
          // dirty 1 is index 1, dirty 2 is index 2
          if (sensor_data[i].beams[0].dirty_1) {
            count_dirty++;
            sensor_data[i].beams[0].dirty_1 = 0;
          }
          if (sensor_data[i].beams[0].dirty_2) {
            count_dirty++;
            sensor_data[i].beams[0].dirty_2 = 0;
          }
        }
        if (count_dirty == (NO_PHOTODIODES * 2)) 
        {
          // perform sweeping with first three diodes only
          sweeping_result = perform_sweeping_manufacturing(0, 0, sensor_data[0].beams[0].index_1, sensor_data[0].beams[0].index_2, 1, sensor_data[1].beams[0].index_1, sensor_data[1].beams[0].index_2, 2, sensor_data[2].beams[0].index_1, sensor_data[2].beams[0].index_2);
          if((sweeping_result < (SWEEPING_DISTANCE + SWEEPING_TOLERANCE)) && (sweeping_result > (SWEEPING_DISTANCE - SWEEPING_TOLERANCE)))
          {
            Serial.print("[SWEEPING_TEST] PASSED WITH ");
            Serial.print(sweeping_result);
            Serial.println(" m");
            test_results[5] = 1;
            pixels.setPixelColor(5, COL_GREEN);
            pixels.show();
            finish_tests();
          }
          else
          {
            Serial.print("[SWEEPING_TEST] FAILED WITH ");
            Serial.print(sweeping_result);
            Serial.println(" m");
            pixels.setPixelColor(5, COL_RED);
            pixels.show();
            Serial.println(" m");
            finish_tests();
          }
        }
        last_timestamp = millis();
      }
      break;

    default:

      break;
  }
}
