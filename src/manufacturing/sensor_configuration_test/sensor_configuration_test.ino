/*
 * L-Cube Sensor Configuration Test
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

// HOW TO USE:
// PLACE TILE IN VIEWING RANGE OF BASE STATION 

// HOW THE HARDWARE SHOULD BEHAVE:
// First, all LEDS should turn pink, to clarify if they are working
// Then, during configuration of the sensors, two red leds next to the corner where a sensor is, indicate that it is not configured so far
// Two green LEDs indicate that this corner is working.
// Then, when the configuration is successful, all LEDs light in green
// Now the user button in the middle should be pressed, and if the signal is captured by the rp2040, all LEDs now blink in green.
// SENSOR 0 IS DIODE 4 ON HARDWARE (REST IST THE SAME INDEX)

#include "sensor_config.h"
#include "config.h"
#include <Adafruit_NeoPixel.h>


uint8_t photodiode_d_pin[MAX_PHOTODIODES] = {D_PINS};
uint8_t photodiode_e_pin[MAX_PHOTODIODES] = {E_PINS};

// class for operating front ws2812 leds
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_LEDS, PIN_LEDS, NEO_GRB + NEO_KHZ800);

void setup() {
  // start serial connection to pc
  Serial.begin(115200);

  #if (DBG_SERIAL == 1)
    // wait for serial connection
    while (!Serial) {}
  #endif

  Serial.println("===========================================================================");
  Serial.println("[EXTENDED_CONFIGURATION_TEST] Serial port connected");

  pinMode(PIN_USERBUTTON, INPUT_PULLUP);

  Serial.println("[EXTENDED_CONFIGURATION_TEST] Turning on builtin LED");
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  Serial.println("[EXTENDED_CONFIGURATION_TEST] Turning on front LEDs");
  pixels.begin();

  // set all pixels to pink to indicate start
  for (int i = 0; i < NUM_LEDS; i++) 
  {
    pixels.setPixelColor(i, COL_PINK);
  }
  pixels.show();

  Serial.println("[EXTENDED_CONFIGURATION_TEST] All LEDs should all light in pink now");
  delay(DELAY_TIME);
  delay(DELAY_TIME);
  delay(DELAY_TIME);

  // ------------------------------------------------------------------------------------------------------------------------
  Serial.println("===========================================================================");
  Serial.println("[EXTENDED_CONFIGURATION_TEST] Starting extended configuration test");

  // only returns if all photodiodes are configured
  configure_sensors();

  Serial.println("[EXTENDED_CONFIGURATION_TEST] Photodiodes are working, now press user button!");

  while(1)
  {
    if(digitalRead(PIN_USERBUTTON) == HIGH)
    {
      break;
    }
  }
  while(1)
  {
    if(digitalRead(PIN_USERBUTTON) == LOW)
    {
      break;
    }
  }

  Serial.println("[EXTENDED_CONFIGURATION_TEST] SUCCESS!!!");

  // blink in green if configuration and userbutton successful
  while(1)
  {
    for (int i = 0; i < NUM_LEDS; i++) 
    {
      if(i % 2 == 0)
      {
        pixels.setPixelColor(i, COL_GREEN);
      }
      else
      {
        pixels.setPixelColor(i, COL_OFF);
      }
    }
    pixels.show();
    delay(DELAY_TIME);
    
    for (int i = 0; i < NUM_LEDS; i++) 
    {
      if(i % 2 == 1)
      {
        pixels.setPixelColor(i, COL_GREEN);
      }
      else
      {
        pixels.setPixelColor(i, COL_OFF);
      }
    }
    pixels.show();
    delay(DELAY_TIME);
  }
}

// ======================================================================================================================================================================
// === MAIN PROGRAM LOOP ===
// ======================================================================================================================================================================
void loop()
{
}
