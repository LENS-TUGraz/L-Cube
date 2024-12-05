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

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "esp_config.h"
#include "esp_mmath.h"

// wifi ssid
extern const char* ssid;
// wifi password
extern const char* password;
// mqtt server domain
extern const char* mqtt_server;
// mqtt client class
extern PubSubClient mqtt_client;
// current operation mode of the esp
extern enum EspOperationMode esp_operation_mode;
// flag to send mqtt_command
extern uint8_t first_time;
// current mqtt_command to transmit to tile
extern char mqtt_command;
// flag that config data was recieved and needs to be forwarded to tile
extern uint8_t send_configuration_to_tiles;
// the buffer which gets send via i2c
extern char i2c_send_buffer[CONFIG_SIZE];


// setup wifi connection
void setup_wifi()
{
  // short delay before connection
  delay(10);

  // connecting to WiFi network
  #if (DBG_MQTT == 1)
    Serial.print("[DBG_MQTT] Connecting to WiFi ");
    Serial.println(ssid);
  #endif
  WiFi.begin(ssid, password);

  // wait for established connection
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    #if (DBG_MQTT == 1)
      Serial.print(".");
    #endif
  }

  #if (DBG_MQTT == 1)
    Serial.println("[DBG_MQTT] WiFi connected");
    Serial.print("[DBG_MQTT] IP address: ");
    Serial.println(WiFi.localIP());
  #endif
}

// connect or reconnect to mqtt broker
void reconnect_mqtt()
{
  // wait until connection or reconnection
  while (!mqtt_client.connected()) {
    #if (DBG_MQTT == 1)
      Serial.print("[DBG_MQTT] Attempting MQTT connection...");
    #endif
    // attempt to connect
    if (mqtt_client.connect(MQTT_CLIENT_NAME))
    {
      #if (DBG_MQTT == 1)
        Serial.println("connected");
      #endif

      // perform subscriptions on topics to receive
      // for initiate config
      mqtt_client.subscribe("c");
      // for initiate quick config
      mqtt_client.subscribe("q");
      // for configuration data
      mqtt_client.subscribe("bs_data");
      // for stop command
      mqtt_client.subscribe("s");
      // normal config commands (1-4 are positions, 5 is to wait for config data)
      mqtt_client.subscribe("1");
      mqtt_client.subscribe("2");
      mqtt_client.subscribe("3");
      mqtt_client.subscribe("4");
      mqtt_client.subscribe("5");
      
      // receives all topics (debugging only)
      //mqtt_client.subscribe("#");
    }
    else
    {
      #if (DBG_MQTT == 1)
        Serial.print("failed, rc=");
        Serial.print(mqtt_client.state());
        Serial.println(" try again in 5 seconds");
      #endif
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

// callback for data that is received via mqtt
void callback_mqtt(char* topic, byte* message, unsigned int length) 
{
  if(strcmp(topic, "bs_data") == 0)
  {
    // switch mode to send configuration
    esp_operation_mode = send_config_data;

    #if (DBG_MQTT == 1)
      // parse config data for printing
      vec3d position = {0};
      mat3x3 heading = {0};
      char command = 0;
      uint8_t station = 0;

      command = message[0];
      station = message[1];
      memcpy(position, &message[2], 12);
      memcpy(heading, &message[14], 36);
      
      Serial.println("[DBG_MQTT] Received Basestation Configuration");
      Serial.print("[DBG_MQTT] Command ");
      Serial.println((char)command);
      Serial.print("[DBG_MQTT] Station ");
      Serial.println(station);
      Serial.println("[DBG_MQTT] Position");
      print_vector(position);
      Serial.println("[DBG_MQTT] Heading");
      print_mat3x3(heading);
    #endif

    // config is 50 bytes long, copy it to i2c buffer and set flag to send it to tiles
    memcpy(i2c_send_buffer, message, CONFIG_SIZE);
    send_configuration_to_tiles = 1;
    return;
  }

  if(strcmp(topic, "c") == 0)
  {
    #if (DBG_MQTT == 1)
      Serial.println("[DBG_MQTT] c command received (config)");
    #endif
    // set esp to config mode
    esp_operation_mode = config_i2c;
    // set MQTT command to send to tile
    mqtt_command = 'c';
    // set flag to only forward command once
    first_time = 1;
    return;
  }

  // forward mqtt commands to tile
  if(strcmp(topic, "1") == 0)
  {
    // first postition
    mqtt_command = '1';
    first_time = 1;
    return;
  }
  if(strcmp(topic, "2") == 0)
  {
    // second postition
    mqtt_command = '2';
    first_time = 1;
    return;
  }
  if(strcmp(topic, "3") == 0)
  {
    // third postition
    mqtt_command = '3';
    first_time = 1;
    return;
  }
  if(strcmp(topic, "4") == 0)
  {
    // fourth postition
    mqtt_command = '4';
    first_time = 1;
    return;
  }

  if(strcmp(topic, "5") == 0)
  {
    // set tile to waiting state for config data
    mqtt_command = '5';
    first_time = 1;
    return;
  }

  if(strcmp(topic, "q") == 0)
  {
    #if (DBG_MQTT == 1)
      Serial.println("[DBG_MQTT] q command received (quick config)");
    #endif
    // set esp to quick config mode
    esp_operation_mode = quick_config_i2c;
    // set command to send to tile
    mqtt_command = 'q';
    // set flag to only forward command once
    first_time = 1;
    return;
  }

  if(strcmp(topic, "s") == 0)
  {
    // set esp to send stop command to tiles
    #if(DBG_MQTT == 1)
      Serial.println("[DBG_MQTT] received stop command");
    #endif
    esp_operation_mode = send_stop_command;
    return;
  }
}

