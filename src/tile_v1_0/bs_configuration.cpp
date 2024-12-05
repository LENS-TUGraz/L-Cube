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

#include "bs_configuration.h"
#include <Adafruit_NeoPixel.h>
#include "config.h"
#include "mmath.h"
#include "positioning.h"


// neopixels
extern Adafruit_NeoPixel pixels;


// flag to rewrite config file
extern uint8_t rewrite_file;
// timestamp for backing configuration in file
extern unsigned long file_timestamp;


// operation mode of the tile
extern enum TileOperationModes operation_mode;
// data representing the decoded beams recevied by the sensors
extern SensorData sensor_data[NO_PHOTODIODES];
// positions of the base stations
extern vec3d station_positions[MAX_BASE_STATIONS];
// orientations of the base stations
extern mat3x3 station_orientations[MAX_BASE_STATIONS];
// station index of received configuration
extern uint8_t config_station_index;


// send buffer for i2c
extern char i2c_send_buffer[QUICK_CONFIG_DATA_BYTES + 1];


// serial data received buffer
uint8_t serial_data[SERIAL_CONFIG_DATA_SIZE] = {0};
// received serial data byte index
uint8_t serial_data_index = 0;
// serial incomming character (c - config)
extern char incoming_serial;


// --- QUICK CONFIG ---
// dirty counter for configuration to check if all sensors received something
uint8_t count_dirty_config = 0;
// councter for correct angle pairs for quick config
uint8_t correct_azimuth_elevation_pairs = 0;
// flag that config data has been sent for the specific station
extern uint8_t config_done[MAX_BASE_STATIONS];
// data for quick configuration
float azimuth_config = 0;
float elevation_config = 0;
float azimuth_quick_config[NO_PHOTODIODES] = {0};
float elevation_quick_config[NO_PHOTODIODES] = {0};


// --- NORMAL CONFIG ---
// state of the ongoing configuration
extern enum ConfigurationSteps config_step;
// current index of logged position
uint16_t config_data_index[MAX_BASE_STATIONS];
// if ready commant should be sent next (recording data finished)
uint8_t send_ready_next = 0;
// start time of the configuration step
long config_start_time = 0;
// contains the latest received command from esp32 for configuration
extern uint8_t last_received_command_number;
// data for normal configuration with averaging
// 6,4kB at the moment for config data
// could malloc this too if to low space
float config_data_azimuth[MAX_BASE_STATIONS][CONFIG_LOGGING_POSITIONS];
float config_data_elevation[MAX_BASE_STATIONS][CONFIG_LOGGING_POSITIONS];
float averaged_azimuth = 0;
float averaged_elevation = 0;


// quick configuration via serial connection
// no averaging of received angles as it is quick config -> if you want a more decent config, use the normal one
void perform_quick_config_serial()
{
  serial_data_index = 0;
  // receive line and check if it was data
  while(Serial.available() > 0)
  {
    // read line and check if it is data
    serial_data[serial_data_index] = Serial.read();

    if(serial_data[serial_data_index] == '\n')
    {
      // line read - check if config data ('d' at beginning and 51 chars long)
      // check for 50, as it gets incremented lated
      if((serial_data[0] == 'd') && (serial_data_index == 50))
      {
        // it's config data, parse it
        config_station_index = serial_data[1];
        memcpy(&station_positions[config_station_index], &serial_data[2], 12);
        memcpy(&station_orientations[config_station_index], &serial_data[14], 36);

        // set flags to rewrite config file
        rewrite_file = 1;
        file_timestamp = millis();

        // turn on light on top to signal config received
        pixels.setPixelColor(1, COL_GREEN);
        pixels.show();
        
        #if (DBG_CONFIG == 1)
          Serial.println("[DBG_CONFIG] got new quick configuration");
          Serial.println("station index");
          Serial.println(config_station_index);
          Serial.println("station position");
          print_vector(station_positions[config_station_index]);
          Serial.println("station orientation");
          print_mat3x3(station_orientations[config_station_index]);
        #endif
      }

      // stop command
      if(serial_data[0] == 's')
      {
        // it's stop command return to normal operation
        operation_mode = normal;
        // turn of config lights
        turn_lights_off();
        return;
      }

      // if config data and \n in it, use it as data
      if(!((serial_data[0] == 'd') && (serial_data_index < 50)))
      {
        // use \n as data
        break;
      }
    }
    
    if(serial_data_index >= 51)
    {
      // don't let buffer overflow happen
      // read rest of buffer and do nothing with it
      while(Serial.available() > 0)
      {
        incoming_serial = Serial.read();
      }
      break;
    }
    else
    {
      serial_data_index++;
    }
  }

  // quick config mode
  for(uint8_t bs_index = 0; bs_index < MAX_BASE_STATIONS; ++bs_index)
  {
    if(config_done[bs_index] == 1)
    {
      // only send config data once for each station
      continue;
    }
    count_dirty_config = 0;

    for(int sensor_index = 0; sensor_index < NO_PHOTODIODES; ++sensor_index)
    {
      // dirty 1 is index 1, dirty 2 is index 2
      if(sensor_data[sensor_index].beams[bs_index].dirty_1 >= 1)
      {
        count_dirty_config++;
        sensor_data[sensor_index].beams[bs_index].dirty_1 = 0;
      }
      if(sensor_data[sensor_index].beams[bs_index].dirty_2 >= 1)
      {
        count_dirty_config++;
        sensor_data[sensor_index].beams[bs_index].dirty_2 = 0;
      }
    }
    if(count_dirty_config == 8)
    {
      // get angles and transfer them via serial to python, where config is calculated
      correct_azimuth_elevation_pairs = 0;

      for(int sensor_index = 0; sensor_index < NO_PHOTODIODES; ++sensor_index)
      {
        if(calculate_azimuth_elevation(0, sensor_data[sensor_index].beams[bs_index].index_1, sensor_data[sensor_index].beams[bs_index].index_2, &azimuth_config, &elevation_config))
        {
          correct_azimuth_elevation_pairs++;
          // save azimuth
          azimuth_quick_config[sensor_index] = azimuth_config;
          // save elevation
          elevation_quick_config[sensor_index] = elevation_config;
        }
      }
      if(correct_azimuth_elevation_pairs == NO_PHOTODIODES)
      {
        // send correct data
        // short for python data (command gets evaluated in python)
        Serial.print("p");
        // station number
        Serial.write(&bs_index, 1);
        for(int sensor_index = 0; sensor_index < NO_PHOTODIODES; ++sensor_index)
        {
          // send azimuth bytes
          Serial.write((byte*)(&azimuth_quick_config[sensor_index]), 4);
          // send elevation bytes
          Serial.write((byte*)(&elevation_quick_config[sensor_index]), 4);
        }
        Serial.println("");
        // flag to only send once -> it is quick config, no need for averaging data
        config_done[bs_index] = 1;
      }
    }
  }
}


// normal configuration via serial connection
void perform_config_serial()
{
  serial_data_index = 0;

  // get commands if we are not in receiving mode
  if(config_step == wait_next_position)
  {
    while(Serial.available() > 0)
    {
      // read line and check if it is data
      serial_data[serial_data_index] = Serial.read();

      if(serial_data[serial_data_index] == '\n')
      {
        if((serial_data[0] >= '1') && (serial_data[0] <= '4'))
        {
          // set according light
          switch(serial_data[0])
          {
            case '1':
              last_received_command_number = 1;
              pixels.setPixelColor(5, COL_BLUE);
              pixels.setPixelColor(7, COL_BLUE);
              pixels.show();
              break;
            case '2':
              last_received_command_number = 2;
              pixels.setPixelColor(1, COL_BLUE);
              pixels.show();
              break;
            case '3':
              last_received_command_number = 3;
              pixels.setPixelColor(3, COL_BLUE);
              pixels.show();
              break;
            case '4':
              last_received_command_number = 4;
              pixels.setPixelColor(5, COL_BLUE);
              pixels.show();
              break;
          }

          // reset index for angle records
          for(int bs_index = 0; bs_index < MAX_BASE_STATIONS; ++bs_index)
          {
            config_data_index[bs_index] = 0;
          }
          // perform recording angles
          config_step = record_data;
          config_start_time = millis();
          
          // do not use old data
          reset_dirty_bits();
          // start logging with next cycle
          return;
        }
        else if(serial_data[0] == '5')
        { 
          // set tile to wait for config data
          config_step = wait_config_data;
          // turn last lights to green
          pixels.setPixelColor(5, COL_GREEN);
          pixels.show();
        }
        break;
      }
      if(serial_data_index >= 51)
      {
        // don't let buffer overflow happen
        // read rest of buffer and do nothing with it
        while(Serial.available() > 0)
        {
          incoming_serial = Serial.read();
        }
        break;
      }
      else
      {
        serial_data_index++;
      }
    }
  }
  else if(config_step == wait_config_data)
  {
    // wait for config data and exit command
    // receive line and check if it was data
    while(Serial.available() > 0)
    {
      // read line and check if it is data
      serial_data[serial_data_index] = Serial.read();

      if(serial_data[serial_data_index] == '\n')
      {
        // line read - check if config data ('d' at beginning and 51 chars long)
        // check for 50, as it gets incremented lated
        if((serial_data[0] == 'd') && (serial_data_index == 50))
        {
          // it's config data, parse it
          config_station_index = serial_data[1];

          memcpy(&station_positions[config_station_index], &serial_data[2], 12);
          memcpy(&station_orientations[config_station_index], &serial_data[14], 36);

          // set flags to rewrite config file
          rewrite_file = 1;
          file_timestamp = millis();
          
          #if (DBG_CONFIG == 1)
            Serial.println("[DBG_CONFIG] got new configuration");
            Serial.println("station index");
            Serial.println(config_station_index);
            Serial.println("station position");
            print_vector(station_positions[config_station_index]);
            Serial.println("station orientation");
            print_mat3x3(station_orientations[config_station_index]);
          #endif
        }

        // stop command
        if(serial_data[0] == 's')
        {
          // it's stop command return to normal operation
          operation_mode = normal;
          // reset state for next config
          config_step = wait_next_position;
          // turn of config lights
          turn_lights_off();
          return;
        }

        // if config data and \n in it, use it as data
        if(!((serial_data[0] == 'd') && (serial_data_index < 50)))
        {
          // use \n as data
          break;
        }
      }
      
      if(serial_data_index >= 51)
      {
        // don't let buffer overflow happen
        // read rest of buffer and do nothing with it
        while(Serial.available() > 0)
        {
          incoming_serial = Serial.read();
        }
        break;
      }
      else
      {
        serial_data_index++;
      }
    }
  }

  switch(config_step)
  {
    case record_data:
      if((millis() - config_start_time) >= CONFIG_LOGGING_TIME)
      {
        config_step = average_and_send_data;
        break;
      }

      // config logging mode
      for(uint8_t bs_index = 0; bs_index < MAX_BASE_STATIONS; ++bs_index)
      {
        // we only record values on sensor 0
        if((sensor_data[0].beams[bs_index].dirty_1 >= 1) && (sensor_data[0].beams[bs_index].dirty_2 >= 1))
        {
          sensor_data[0].beams[bs_index].dirty_1 = 0;
          sensor_data[0].beams[bs_index].dirty_2 = 0;

          // get angles and record them
          if(calculate_azimuth_elevation(0, sensor_data[0].beams[bs_index].index_1, sensor_data[0].beams[bs_index].index_2, &azimuth_config, &elevation_config))
          {
            // record data
            if(config_data_index[bs_index] < CONFIG_LOGGING_POSITIONS)
            {
              config_data_azimuth[bs_index][config_data_index[bs_index]] = azimuth_config;
              config_data_elevation[bs_index][config_data_index[bs_index]] = elevation_config;
              config_data_index[bs_index]++;
            }
          }
        }
      }
      break;

    case average_and_send_data:

      #if (DBG_CONFIG == 1)
        Serial.println("[DBG_CONFIG] Start averaging and sending data to python");
      #endif
      // average over all recorded data and send it to python
      for(uint8_t bs_index = 0; bs_index < MAX_BASE_STATIONS; ++bs_index)
      {
        if(config_data_index[bs_index] < 2)
        {
          // there are at least 2 angles required from one base station to average and send it
          continue;
        }

        // reset averaged values
        averaged_azimuth = 0.0;
        averaged_elevation = 0.0;
        for(uint16_t data_index = 0; data_index < config_data_index[bs_index]; ++data_index)
        {
          averaged_azimuth += config_data_azimuth[bs_index][data_index];
          averaged_elevation += config_data_elevation[bs_index][data_index];
        }
        averaged_azimuth /= (float)config_data_index[bs_index];
        averaged_elevation /= (float)config_data_index[bs_index];

        // send angles to pyhton
        // short for python data (command gets evaluated in python)
        Serial.print("p");
        // station number
        Serial.write(&bs_index, 1);
        // send azimuth bytes
        Serial.write((byte*)(&averaged_azimuth), 4);
        // send elevation bytes
        Serial.write((byte*)(&averaged_elevation), 4);
        // trigger newline
        Serial.println("");
      }
      // send ready command, so all positions have been sent and python can continue to next step
      Serial.println("r");
      // turn according light green
      switch(last_received_command_number)
      {
        case 1:
          pixels.setPixelColor(5, COL_GREEN);
          pixels.show();
          break;
        case 2:
          pixels.setPixelColor(7, COL_GREEN);
          pixels.show();
          break;
        case 3:
          pixels.setPixelColor(1, COL_GREEN);
          pixels.show();
          break;
        case 4:
          pixels.setPixelColor(3, COL_GREEN);
          pixels.show();
          break;
      }

      config_step = wait_next_position;
      break;

    default:
      break;
  }
}

// normal config via i2c
void perform_config_i2c()
{
  switch(config_step)
  {
    case record_data:
      if((millis() - config_start_time) >= CONFIG_LOGGING_TIME)
      {
        config_step = average_and_send_data;
        break;
      }

      // config logging mode
      for(uint8_t bs_index = 0; bs_index < MAX_BASE_STATIONS; ++bs_index)
      {
        // we only record values on sensor 0
        if((sensor_data[0].beams[bs_index].dirty_1 >= 1) && (sensor_data[0].beams[bs_index].dirty_2 >= 1))
        {
          sensor_data[0].beams[bs_index].dirty_1 = 0;
          sensor_data[0].beams[bs_index].dirty_2 = 0;

          // get angles and record them
          if(calculate_azimuth_elevation(0, sensor_data[0].beams[bs_index].index_1, sensor_data[0].beams[bs_index].index_2, &azimuth_config, &elevation_config))
          {
            // record data
            if(config_data_index[bs_index] < CONFIG_LOGGING_POSITIONS)
            {
              config_data_azimuth[bs_index][config_data_index[bs_index]] = azimuth_config;
              config_data_elevation[bs_index][config_data_index[bs_index]] = elevation_config;
              config_data_index[bs_index]++;
            }
          }
        }
      }
      break;

    case average_and_send_data:

      if(send_ready_next == 1)
      {
        // send ready command
        memcpy((void*)(i2c_send_buffer), "r", 1);
        i2c_send_buffer[1] = 0;

        // signal that data is ready via interrupt pin 
        // care to not use rising edge for interrupt with longer spikes, as setting pin from high to low is not a clean edge
        digitalWrite(I2C_INT_TO_ESP, HIGH);
        digitalWrite(I2C_INT_TO_ESP, LOW);

        #if (DBG_I2C == 1)
          Serial.println("[DBG_I2C] send ready command");
        #endif

        send_ready_next = 0;

        config_step = wait_next_position;
        break;
      }

      #if (DBG_CONFIG == 1)
        Serial.println("[DBG_CONFIG] Start averaging and sending data via i2c and mqtt to python");
      #endif
      // average over all recorded data and send it to python
      for(uint8_t bs_index = 0; bs_index < MAX_BASE_STATIONS; ++bs_index)
      {
        if(config_data_index[bs_index] < 2)
        {
          // there are at least 2 angles required from one base station to average and send it
          continue;
        }

        #if (DBG_CONFIG == 1)
          Serial.print("[DBG_CONFIG] got angles for station ");
          Serial.println(bs_index);
        #endif
        
        // reset averaged values
        averaged_azimuth = 0.0;
        averaged_elevation = 0.0;
        for(uint16_t data_index = 0; data_index < config_data_index[bs_index]; ++data_index)
        {
          averaged_azimuth += config_data_azimuth[bs_index][data_index];
          averaged_elevation += config_data_elevation[bs_index][data_index];
        }
        averaged_azimuth /= (float)config_data_index[bs_index];
        averaged_elevation /= (float)config_data_index[bs_index];
        
        // prepare data to send via i2c
        // short for python data (command gets evaluated in python)
        memcpy((void*)(i2c_send_buffer), "p", 1);
        memcpy((void*)(i2c_send_buffer+1), &bs_index, 1);
        memcpy((void*)(i2c_send_buffer+2), &averaged_azimuth, 4);
        memcpy((void*)(i2c_send_buffer+6), &averaged_elevation, 4);
        i2c_send_buffer[NORMAL_CONFIG_DATA_BYTES] = 0;

        // signal that data is ready via interrupt pin 
        // care to not use rising edge for interrupt with longer spikes, as setting pin from high to low is not a clean edge
        digitalWrite(I2C_INT_TO_ESP, HIGH);
        digitalWrite(I2C_INT_TO_ESP, LOW);
        
        
        #if (DBG_I2C == 1)
          Serial.println("[DBG_I2C] sent configuration data");
          
        #endif

        // tile has 20 ms to do i2c communication
        delay(20);
      }

      send_ready_next = 1;

      // turn according light green
      switch(last_received_command_number)
      {
        case 1:
          pixels.setPixelColor(5, COL_GREEN);
          pixels.show();
          break;
        case 2:
          pixels.setPixelColor(7, COL_GREEN);
          pixels.show();
          break;
        case 3:
          pixels.setPixelColor(1, COL_GREEN);
          pixels.show();
          break;
        case 4:
          pixels.setPixelColor(3, COL_GREEN);
          pixels.show();
          break;
      }
      break;

    default:
      break;
  }

}

// quick configuration via mqtt and i2c
// no averaging of received angles as it is quick config -> if you want a more decent config, use the normal one
void perform_quick_config_i2c()
{
  // quick config mode
  for(uint8_t bs_index = 0; bs_index < MAX_BASE_STATIONS; ++bs_index)
  {
    if(config_done[bs_index] == 1)
    {
      // only send config data once for each station
      continue;
    }
    count_dirty_config = 0;

    for(int sensor_index = 0; sensor_index < NO_PHOTODIODES; ++sensor_index)
    {
      // dirty 1 is index 1, dirty 2 is index 2
      if(sensor_data[sensor_index].beams[bs_index].dirty_1 >= 1)
      {
        count_dirty_config++;
        sensor_data[sensor_index].beams[bs_index].dirty_1 = 0;
      }
      if(sensor_data[sensor_index].beams[bs_index].dirty_2 >= 1)
      {
        count_dirty_config++;
        sensor_data[sensor_index].beams[bs_index].dirty_2 = 0;
      }
    }
    if(count_dirty_config == 8)
    {
      // get angles and transfer them via serial to python, where config is calculated
      correct_azimuth_elevation_pairs = 0;

      for(int sensor_index = 0; sensor_index < NO_PHOTODIODES; ++sensor_index)
      {
        if(calculate_azimuth_elevation(0, sensor_data[sensor_index].beams[bs_index].index_1, sensor_data[sensor_index].beams[bs_index].index_2, &azimuth_config, &elevation_config))
        {
          correct_azimuth_elevation_pairs++;
          // save azimuth
          azimuth_quick_config[sensor_index] = azimuth_config;
          // save elevation
          elevation_quick_config[sensor_index] = elevation_config;
        }
      }
      if(correct_azimuth_elevation_pairs == NO_PHOTODIODES)
      {
        // prepare data to send via i2c:
        // short for python data (command gets evaluated in python)
        memcpy((void*)(i2c_send_buffer), "p", 1);
        memcpy((void*)(i2c_send_buffer+1), &bs_index, 1);
        for(int sensor_index = 0; sensor_index < NO_PHOTODIODES; ++sensor_index)
        {
          // prepare azimuth bytes
          memcpy((void*)(i2c_send_buffer + 2 + (sensor_index * 8)), &azimuth_quick_config[sensor_index], 4);
          memcpy((void*)(i2c_send_buffer + 6 + (sensor_index * 8)), &elevation_quick_config[sensor_index], 4);
        }
        i2c_send_buffer[QUICK_CONFIG_DATA_BYTES] = 0;

        // signal that data is ready via interrupt pin 
        // care to not use rising edge for interrupt with longer spikes, as setting pin from high to low is not a clean edge
        digitalWrite(I2C_INT_TO_ESP, HIGH);
        digitalWrite(I2C_INT_TO_ESP, LOW);

        // tile has 20 ms to do i2c communication
        delay(20);
        #if (DBG_I2C == 1)
          Serial.println("[DBG_I2C] sent configuration data");
        #endif
        
        // flag to only send once
        config_done[bs_index] = 1;
      }
    }
  }
}

void start_normal_config_record()
{
  #if (DBG_CONFIG == 1)
    Serial.println("[DBG_CONFIG] Start logging i2c normal data at current position");
  #endif
  
  // reset index for angle records
  for(int bs_index = 0; bs_index < MAX_BASE_STATIONS; ++bs_index)
  {
    config_data_index[bs_index] = 0;
  }
  // start recording angles
  config_step = record_data;
  // set start time
  config_start_time = millis();
  // do not use old data
  reset_dirty_bits();
}


// turns off all front lights
void turn_lights_off()
{
  for (int i = 0; i < NUM_LEDS; i++) 
  {
    pixels.setPixelColor(i, COL_OFF);
  }
  pixels.show();
}


void reset_dirty_bits()
{
  for(int sensor_index = 0; sensor_index < NO_PHOTODIODES; ++sensor_index)
  {
    for(int bs_index = 0; bs_index < MAX_BASE_STATIONS; ++bs_index)
    {
      sensor_data[sensor_index].beams[bs_index].dirty_1 = 0;
      sensor_data[sensor_index].beams[bs_index].dirty_2 = 0;
    }
  }
}
