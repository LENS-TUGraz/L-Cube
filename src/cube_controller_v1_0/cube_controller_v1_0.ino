/*
 * L-Cube Cube Controller Software
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

#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>


#include "esp_mmath.h"
#include "esp_config.h"
#include "esp_isr.h"
#include "esp_mqtt.h"
#include "esp_positioning.h"


// - MQTT variables -----------------------------------
// wifi ssid
const char* ssid = WIFI_SSID;
// wifi password
const char* password = WIFI_PASSWORD;
// mqtt server domain
const char* mqtt_server = MQTT_SERVER;
// wifi client class
WiFiClient wifi_client;
// mqtt client class
PubSubClient mqtt_client(wifi_client);

// - positioning variables ----------------------------
// contains position and heading of the lost cube
CubeData cube_data;
// contains position and heading of the tiles
TileData tile_data[NO_TILES];
// timestamp for positioning interval
long last_timestamp = 0;

// - control flow variables ---------------------------
// current operation mode of the esp
enum EspOperationMode esp_operation_mode = normal;
// flag that config data was recieved and needs to be forwarded to tile
uint8_t send_configuration_to_tiles = 0;
// flag that tile wants to send data via i2c
uint8_t i2c_flag[NO_TILES];
// set flag to only send command for configs for one time
uint8_t first_time = 0;
// current mqtt_command to transmit to tile
char mqtt_command = 0;

// - isr variables ------------------------------------
// i2c isr declarations
void (*i2c_isrs[NO_TILES])() = { isr_i2c0, isr_i2c1, isr_i2c2, isr_i2c3, isr_i2c4};

// - pin variables ------------------------------------
// pins for interrupts when a tile wants to send data
uint8_t i2c_com_request_pin[NO_TILES] = {I2C_COM_REQUEST_PINS};

// - i2c variables ------------------------------------
// python MQTT config data that is published
char config_data[MQTT_QUICK_CONFIG_DATA_BYTES + 1] = {0};
// i2c send buffer for config
char i2c_send_buffer[CONFIG_SIZE] = {0};
// i2c receive buffer for config
uint8_t i2c_receive_buffer[MQTT_QUICK_CONFIG_DATA_BYTES +1] = {0};
// i2c index of received byte for config
uint32_t i2c_receive_index = 0;
// positioning updates from tiles are received in this buffer
char i2c_position_update_data[I2C_POSITION_UPDATE_SIZE + 1] = {0};		
// index of current received byte for position updates
uint32_t i2c_position_update_receive_index = 0;

void setup() {
  Serial.begin(115200);

  #if (DBG_SERIAL == 1)
    while(!Serial){}
    Serial.println("[DBG_SERIAL] Serial connection established");
  #endif
  
  for(int i = 0; i < NO_TILES; ++i)
  {
    // set input pins for i2c
    pinMode(i2c_com_request_pin[i], INPUT);
  }

  #if (USE_MQTT == 1)
    // enable external antenna
    pinMode(3, OUTPUT);    // RF switch power on
    digitalWrite(3, LOW);
    pinMode(14, OUTPUT);   // select external antenna
    digitalWrite(14, HIGH);

    setup_wifi();
    mqtt_client.setServer(mqtt_server, 1883);
    mqtt_client.setCallback(callback_mqtt);
  #endif

  // setup i2c
  Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQUENCY); // join i2c bus (address optional for master)

  // attach interrupts for i2c communication request pins
  for(int i = 0; i < NO_TILES; ++i)
  {
    // set interrupts for i2c
    attachInterrupt(digitalPinToInterrupt(i2c_com_request_pin[i]), i2c_isrs[i], FALLING);
  }

  last_timestamp = millis();

  #if (DBG_SERIAL == 1)
    while(!Serial){}
    Serial.println("[DBG_SERIAL] Setup completed");
  #endif
}

#if (SERIAL_MODE == 1)
  // buffer for data
  uint8_t serial_data[MAX_SERIAL_DATA_LENGTH] = {0};
  // index of incomming byte
  uint16_t serial_data_index = 0;
  // for overflow
  uint8_t incoming_serial = 0;
#endif

#if (TEST_MODE == 1)
  // test data from tiles
  uint8_t cube_test_data1[TEST_DATA_SIZE];
  uint8_t cube_test_data2[TEST_DATA_SIZE];
  uint8_t cube_test_data3[TEST_DATA_SIZE];
  uint8_t cube_test_data4[TEST_DATA_SIZE];
  uint8_t cube_test_data5[TEST_DATA_SIZE];

  uint8_t *test_data[5] = {cube_test_data1, cube_test_data2, cube_test_data3, cube_test_data4, cube_test_data5};
#endif


// # loop function ####################################################################################################################
void loop() {

  #if (USE_MQTT == 1)
    // asure mqtt connection
    if (!mqtt_client.connected()) {
      reconnect_mqtt();
    }

    // call mqtt internal loop
    mqtt_client.loop();
    // until here, mqtt needs up to 3ms, so should be fine (included sending bs_data)
  #endif
  
  #if (SERIAL_MODE == 1)
    // get serial data (configuration) if available
    if(Serial.available() > 0)
    {
      serial_data_index = 0;
      // receive line and check if it was data
      while(Serial.available() > 0)
      {
        // read line and check if it is data
        serial_data[serial_data_index] = Serial.read();

        if(serial_data[serial_data_index] == '\n')
        {
          // line read - check if config data ('d' at beginning and 50 chars long)
          // check for 50, as it gets incremented later
          if((serial_data[0] == 'd') && (serial_data_index == 50))
          {
            // it's config data, forward it
            for(uint8_t i = 0; i < NO_TILES; ++i)
            {
              // send configuration to tile
              Wire.beginTransmission(I2C_ADDRESS_OFFSET + i);
              Wire.write(serial_data, CONFIG_SIZE);
              Wire.endTransmission();
            }
          }

          // stop command
          if(serial_data[0] == 's')
          {
            // it's stop command, forward it
            char stop = 's';
            Serial.println("stp_signal");
            // send stop command
            for(uint8_t i = 0; i < NO_TILES; ++i)
            {
              Wire.beginTransmission(I2C_ADDRESS_OFFSET + i);
              Wire.write(stop);
              Wire.endTransmission();
            }
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
  #endif

  // perform different modes of operation
  switch(esp_operation_mode)
  {
  // ==========================================================================================================
    case normal:
      // perform positioning
      for(uint8_t tile_index = 0; tile_index < NO_TILES; ++tile_index)
      {
        // receive or send tile data if required
        if(i2c_flag[tile_index] == 1)
        {
          // check if all tiles can send data (only required after assembly)
          //Serial.println(tile_index);
          //Serial.println(i2c_position_update_data[0]);

          // reset i2c request flag
          i2c_flag[tile_index] = 0;
          // receive positioning data from tile
          #if (DBG_I2C == 1)
            //Serial.print("[DBG_I2C] Starting transaction to tile ");
            //Serial.println(tile_index);
          #endif

          // perform i2c transfer
          // receive data via i2c
          Wire.requestFrom(I2C_ADDRESS_OFFSET + tile_index, I2C_POSITION_UPDATE_SIZE);
          i2c_position_update_receive_index = 0;
          while (Wire.available())
          {
            // receive i2c data
            i2c_position_update_data[i2c_position_update_receive_index] = Wire.read();
            i2c_position_update_receive_index++;
          }
						 
          // u for crossing update, v for sweeping update
          if((i2c_position_update_data[0] != 'u') && (i2c_position_update_data[0] != 'v'))
          {
            // no valid data, bus error
            continue;
          }

          #if (TEST_MODE == 1)
            // for testing, data is forwarded to serial here
            memcpy(test_data[tile_index], i2c_position_update_data, TEST_DATA_SIZE);
            
            // crossing update
            if(i2c_position_update_data[0] == 'u')
            {
              test_data[tile_index][0] = tile_index + 20;
            }
            // sweeping update
            if(i2c_position_update_data[0] == 'v')
            {
              test_data[tile_index][0] = tile_index;
            }
            
            
            Serial.write((byte*)test_data[tile_index], TEST_DATA_SIZE);
            Serial.println("");
            
            // could also send data to pc via mqtt, but cable is more reliable
            // mqtt_client.publish("t", test_data[tile_index], TEST_DATA_SIZE);
          #endif

          memcpy((void*)tile_data[tile_index].tile_position, (void*)(&i2c_position_update_data[1]), 12);
          memcpy((void*)tile_data[tile_index].forward_vector, (void*)(&i2c_position_update_data[13]), 12);
          memcpy((void*)tile_data[tile_index].up_vector, (void*)(&i2c_position_update_data[25]), 12);
          
          // set sweeping or crossing (u for crossing, v for sweeping)
          if(i2c_position_update_data[0] == 'u')
          {
            tile_data[tile_index].is_crossing = 1;
          }
          if(i2c_position_update_data[0] == 'v')
          {
            tile_data[tile_index].is_crossing = 0;
          }

          // set timestamp
          tile_data[tile_index].last_update = millis();

          #if (DBG_I2C == 1)
            Serial.print("[DBG_I2C] Received data from tile ");
            Serial.println(tile_index);
            Serial.println("[DBG_I2C] Position");
            print_vector(tile_data[tile_index].tile_position);
            Serial.println("[DBG_I2C] Forward Vector");
            print_vector(tile_data[tile_index].forward_vector);
            Serial.println("[DBG_I2C] Up Vector");
            print_vector(tile_data[tile_index].up_vector);
          #endif
        }
      }

      if((millis() - last_timestamp) > UPDATE_PERIOD)
      {
        // calculate global position for cube
        update_global_position();
        last_timestamp = millis();
      }
      break;

  // ==========================================================================================================
    case config_i2c:
      // config only possible with top tile
      if(first_time == 1)
      {
        #if (DBG_I2C == 1)
          Serial.print("[DBG_I2C] send command via i2c: ");
          Serial.println(mqtt_command);
        #endif

        first_time = 0;

        // send mqtt_command to tile
        Wire.beginTransmission(I2C_ADDRESS_OFFSET + CONFIG_TILE_INDEX);
        Wire.write(mqtt_command);
        Wire.endTransmission();
      }

      // tile wants to send config data
      if(i2c_flag[CONFIG_TILE_INDEX] == 1)
      {
        // receive config data from tile
        #if (DBG_I2C == 1)
          Serial.println("[DBG_I2C] Starting receiving normal config data from top tile");
        #endif

        // receive data via i2c
        Wire.requestFrom(I2C_ADDRESS_OFFSET + CONFIG_TILE_INDEX, MQTT_NORMAL_CONFIG_DATA_BYTES);
        i2c_receive_index = 0;

        while (Wire.available())
        {
          // receive i2c data
          i2c_receive_buffer[i2c_receive_index] = Wire.read();
          i2c_receive_index++;
        }

        // early return when received command is "ready"
        if(i2c_receive_buffer[0] == 'r')
        {
          // ready command -> publish it
          #if (DBG_I2C == 1)
            Serial.println("[DBG_I2C] Publish ready command");
          #endif
          mqtt_client.publish("r","1");

          i2c_flag[CONFIG_TILE_INDEX] = 0;
          break;
        }

        // received data is config data - copy to outgoing mqtt stream
        memcpy((void*)config_data, (void*)(i2c_receive_buffer), MQTT_NORMAL_CONFIG_DATA_BYTES);
        
        #if (DBG_I2C == 1)
          Serial.println("[DBG_I2C] Received config data");
          Serial.println("[DBG_I2C] Command");
          Serial.println((char)i2c_receive_buffer[0]);
          Serial.println("[DBG_I2C] Station");
          Serial.println((uint8_t)i2c_receive_buffer[1]);
          Serial.println("[DBG_I2C] Azimuth & Elevation");
          Serial.print("[");
          Serial.print(*((float*)(&i2c_receive_buffer[2])));
          Serial.print(",");
          Serial.print(*((float*)(&i2c_receive_buffer[6])));
          Serial.println("]");
        #endif

        // transfer data to pc via MQTT
        // p for python data
        mqtt_client.publish("p", (uint8_t*)config_data, MQTT_NORMAL_CONFIG_DATA_BYTES);

        i2c_flag[CONFIG_TILE_INDEX] = 0;
      }
      break;

  // ==========================================================================================================
    case quick_config_i2c:
      // flag for only forward command once
      if(first_time == 1)
      {
        #if (DBG_I2C == 1)
          Serial.print("[DBG_I2C] send command via i2c: ");
          Serial.println(mqtt_command);
        #endif
        first_time = 0;

        // send mqtt_command to tile
        Wire.beginTransmission(I2C_ADDRESS_OFFSET + CONFIG_TILE_INDEX);
        Wire.write(mqtt_command);
        Wire.endTransmission();
      }

      // tile now records data, wait for interrupts and then forward data to python
      if(i2c_flag[CONFIG_TILE_INDEX] == 1)
      {
        // receive config data from tile
        #if (DBG_I2C == 1)
          Serial.println("[DBG_I2C] Starting receiving quick config data from top tile");
        #endif

        // receive data via i2c
        Wire.requestFrom(I2C_ADDRESS_OFFSET + CONFIG_TILE_INDEX, MQTT_QUICK_CONFIG_DATA_BYTES);
        i2c_receive_index = 0;

        while (Wire.available())
        {
          // receive i2c data
          i2c_receive_buffer[i2c_receive_index] = Wire.read();
          i2c_receive_index++;
        }

        // copy received config data to outgoing mqtt stream
        memcpy((void*)config_data, (void*)(i2c_receive_buffer), MQTT_QUICK_CONFIG_DATA_BYTES);
        
        #if (DBG_I2C == 1)
          Serial.println("[DBG_I2C] Received config data");
          Serial.println("[DBG_I2C] Command");
          Serial.println((char)i2c_receive_buffer[0]);
          Serial.println("[DBG_I2C] Station");
          Serial.println((uint8_t)i2c_receive_buffer[1]);
          Serial.println("[DBG_I2C] Azimuths & Elevations (a1, e1, a2, e2, ...");
          for(int i = 0; i < 8; ++i)
          {
            Serial.println(*((float*)(&i2c_receive_buffer[2 + (i * 4)])));
          }
        #endif

        #if (USE_MQTT == 1)
          // transfer data to pc via MQTT
          // p for python data
          mqtt_client.publish("p", (uint8_t*)config_data, MQTT_QUICK_CONFIG_DATA_BYTES);
        #endif

        i2c_flag[CONFIG_TILE_INDEX] = 0;
      }
      
      break;

  // ==========================================================================================================
    case send_config_data:
      if(send_configuration_to_tiles == 1)
      {
        // send data to tiles
        for(uint8_t i = 0; i < NO_TILES; ++i)
        {
          // send mqtt_command to tile
          Wire.beginTransmission(I2C_ADDRESS_OFFSET + i);
          Wire.write((uint8_t*)i2c_send_buffer, CONFIG_SIZE);
          Wire.endTransmission();
        }
        send_configuration_to_tiles = 0;
      }
      break;

// ==========================================================================================================
    case send_stop_command:
      // send stop command to tiles and continue in normal mode
      #if (DBG_I2C)
        Serial.println("[DBG_I2C] send stop command to tile");
      #endif
      char stop = 's';

      // send stop command
	    for(uint8_t i = 0; i < NO_TILES; ++i)
      {
        Wire.beginTransmission(I2C_ADDRESS_OFFSET + i);
        Wire.write(stop);
        Wire.endTransmission();
	    }

      esp_operation_mode = normal;
      break;
  }
}

