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

#include <ts4231.h>
#include "config.h"
#include "sensor_config.h"
#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

extern uint8_t photodiode_d_pin[MAX_PHOTODIODES];
extern uint8_t photodiode_e_pin[MAX_PHOTODIODES];

TS4231* sensors[MAX_PHOTODIODES];

uint8_t photodiode_configured[MAX_PHOTODIODES] = {0};

extern uint pio_state_machines[8];
extern Adafruit_NeoPixel pixels;

void configure_sensors()
{

  // set all pixels to red to indicate start
  for (int i = 0; i < NUM_LEDS; i++) 
  {
    pixels.setPixelColor(i, COL_RED);
  }
  pixels.show();

  uint8_t configuration_missing = 1;
  uint8_t config_result = 0;

  // construct device classes
  for(int i = 0; i < NO_PHOTODIODES; ++i)
  {
    sensors[i] = new TS4231(photodiode_e_pin[i], photodiode_d_pin[i]);
  }

  while(configuration_missing)
  {
    // configure the photodiodes
    for(int i = 0; i < NO_PHOTODIODES; ++i)
    {
      if(photodiode_configured[i] == 0)
      {
        // wait for light first, as this is the way how to configure
        if (sensors[i]->waitForLight(LIGHT_TIMEOUT))
        {
          #if (DBG_SENSORS == 1)
            Serial.print("[DBG_SENSORS] Light for Sensor ");
            Serial.print(i);
            Serial.println(" DETECTED");
          #endif
          
          // other methods might not work because of the issue that 
          // rpi sets the output to low if configured as output by default
          // this may then violate timings
          config_result = sensors[i]->configDevice();

          // when a fail state occurs, just try again, otherwise set flag
          if(config_result == CONFIG_PASS)
          {
            photodiode_configured[i] = 1;
            if(i == 0)
            {
              pixels.setPixelColor(0, COL_GREEN);
              pixels.setPixelColor(2, COL_GREEN);
              pixels.show();
            }
            else if(i == 1)
            {
              pixels.setPixelColor(1, COL_GREEN);
              pixels.setPixelColor(7, COL_GREEN);
              pixels.show();
            }
            else if(i == 2)
            {
              pixels.setPixelColor(5, COL_GREEN);
              pixels.setPixelColor(3, COL_GREEN);
              pixels.show();
            }
            else if(i == 3)
            {
              pixels.setPixelColor(6, COL_GREEN);
              pixels.setPixelColor(4, COL_GREEN);
              pixels.show();
            }
            Serial.print("Sensor ");
            Serial.print(i);
            Serial.println(" successful configured");
          }

          #if (DBG_SENSORS == 1)
            // print configuration status
            Serial.print("[DBG_SENSORS] Sensor ");
            Serial.print(i);
            switch (config_result) 
            {
              case CONFIG_PASS:
                Serial.println(" Configuration SUCCESS");
                break;
              case BUS_FAIL: // unable to resolve state of TS4231 (3 samples of the bus signals resulted in 3 different states)
                Serial.println(" Configuration Unsuccessful - BUS_FAIL");
                break;
              case VERIFY_FAIL: // configuration read value did not match configuration write value, run configuration again
                Serial.println(" Configuration Unsuccessful - VERIFY_FAIL");
                break;
              case WATCH_FAIL: //verify succeeded but entry into WATCH mode failed, run configuration again
                Serial.println(" Configuration Unsuccessful - WATCH_FAIL");
                break;
              default: //value returned was unknown
                Serial.println(" Program Execution ERROR");
                break;
            }
          #endif
        }
        else
        {
          #if (DBG_SENSORS == 1)
            // no light detection
            Serial.print("[DBG_SENSORS] Light Sensor ");
            Serial.print(i);
            Serial.println(" TIMEOUT");
          #endif
        }
      }
    }

    // check if all photodiodes are configured
    configuration_missing = 0;
    for(int i = 0; i < NO_PHOTODIODES; ++i)
    {
      if(photodiode_configured[i] == 0)
      {
        configuration_missing = 1;
      }
    }
  }
}
