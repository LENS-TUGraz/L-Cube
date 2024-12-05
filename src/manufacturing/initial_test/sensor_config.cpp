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

#include <ts4231.h>
#include "config.h"
#include "sensor_config.h"
#include <hardware/pio.h>
#include "differential_manchester.pio.h"
#include <Arduino.h>

extern uint8_t photodiode_d_pin[MAX_PHOTODIODES];
extern uint8_t photodiode_e_pin[MAX_PHOTODIODES];

TS4231* sensors[MAX_PHOTODIODES];

uint8_t photodiode_configured[MAX_PHOTODIODES] = {0};

extern uint pio_state_machines[8];

void configure_sensors()
{
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


void setup_pio()
{
  #if (DBG_SENSORS == 1)
    Serial.println("[DBG_SENSORS] Before adding program");
  #endif

  uint offset_rx1 = 0;

  if(pio_can_add_program(pio1, &differential_manchester_rx_program))
  {
    offset_rx1 = pio_add_program(pio1, &differential_manchester_rx_program);
    #if (DBG_SENSORS == 1)
      Serial.print("[DBG_SENSORS] Receive PIO1 program loaded at ");
      Serial.println(offset_rx1);
    #endif
  }
  else
  {
    #if (DBG_SENSORS == 1)
      Serial.println("[DBG_SENSORS] Not able to add programm memory of pio1");
    #endif
  }

  // Configure state machines, set bit rate at 6 Mbps
  // 125MHz default clock (even if arduino selected 133MHz) -> dividing by 125 makes 1MHz as timebase, times 16 (2*8) as diff manchester has twice the clock speed as bit rate
  // and we want MB -> times 6 for 6 MHz
  float clock_divider = 125.f / (16 * 6); // this might have to be adapted to 133MHz frequenzy
  
  // init pio program on state machine -> only that line should then be done more often (each sm has own pc)
  for(int i = 0; i < NO_PHOTODIODES; ++i)
  {
    differential_manchester_rx_program_init(pio1, pio_state_machines[i], offset_rx1, photodiode_d_pin[i], photodiode_e_pin[i], clock_divider);
  }

  #if (DBG_SENSORS == 1)
    Serial.print("[DBG_SENSORS] PIO loaded");
  #endif

}
