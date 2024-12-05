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

#include "i2c.h"
#include <Arduino.h>
#include <Wire.h>
#include "config.h"

// operation mode of the tile
extern enum TileOperationModes operation_mode;

// i2c address of tile
extern uint8_t i2c_address;
// flag for i2c message was received
extern uint8_t i2c_received;
// if current config mode is quick config
extern uint8_t quick_config_mode;

// received byte index of i2c
uint16_t i2c_receive_index = 0;
// receive buffer of i2c
extern char i2c_receive_buffer[CONFIG_SIZE + 1];
// send buffer of i2c for config
extern char i2c_send_buffer[QUICK_CONFIG_DATA_BYTES + 1];
// send buffer of i2c for position updates
extern char i2c_update_data[UPDATE_SIZE + 1];

// send position and heading to esp
void i2c_send_positioning_update(vec3d pos, vec3d fw_vector, vec3d u_vector, uint8_t localization_mode)
{
	// copy data to stream buffer
  if(localization_mode == 0)
  {
    // crossing update
    i2c_update_data[0] = 'u';
  }
  else
  {
    // sweeping update
    i2c_update_data[0] = 'v';
  }
  memcpy((i2c_update_data+1), pos, 12);
  memcpy((i2c_update_data+13), fw_vector, 12);
  memcpy((i2c_update_data+25), u_vector, 12);

  // signal that data is ready via interrupt pin 
  // care to not use rising edge for interrupt with longer spikes, as setting pin from high to low is not a clean edge
  digitalWrite(I2C_INT_TO_ESP, HIGH);
  digitalWrite(I2C_INT_TO_ESP, LOW);

  #if ((DBG_POSITION == 1) || (DBG_I2C == 1))
    Serial.println("[DBG_I2C|DBG_POSITION] send position update");
  #endif
}


// i2c callback for receiving data
void receive_i2c_data(int count)
{
  i2c_receive_index = 0;
  while(Wire1.available())
  {
    if(i2c_receive_index > CONFIG_SIZE)
    {
      #if (DBG_I2C == 1)
        Serial.println("[DBG_I2C] ERROR, to long packet");
      #endif
    }
    // receive data
    i2c_receive_buffer[i2c_receive_index] = Wire1.read();
    i2c_receive_index++;
  }
  i2c_received = 1;
}


// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void send_i2c_data()
{
  if(operation_mode == normal)
  {
    Wire1.write(i2c_update_data, UPDATE_SIZE);
  }
  else
  {
    // based on first command, send according data size
    if(i2c_send_buffer[0] == 'r')
    {
      Wire1.write(i2c_send_buffer, NORMAL_CONFIG_DATA_BYTES);
    }
    else if(i2c_send_buffer[0] == 'p')
    {
      if(quick_config_mode == 1)
      {
        Wire1.write(i2c_send_buffer, QUICK_CONFIG_DATA_BYTES);
      }
      else
      {
        Wire1.write(i2c_send_buffer, NORMAL_CONFIG_DATA_BYTES);
      }
    }
  }
}



