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

// includeguard
#ifndef ESP_CONFIG_
#define ESP_CONFIG_

// = SERIAL =================================================================
// wait with setup until serial connection to pc is established and print debugs to serial
#define DBG_SERIAL 0

// = TEST AND SERIAL MODE (CONNECT CUBE WITH CABLE TO PC) ===================
// supports running cube via cable and visualize it
#define SERIAL_MODE 1
// forwards tile data via serial to pc - SERIAL_MODE needs to be activated too!
#define TEST_MODE 0
// max length of serial data received in serial mode
#define MAX_SERIAL_DATA_LENGTH 51
// size of data to forward from tile to pc
#define TEST_DATA_SIZE 37

// = MQTT ===================================================================
// enable debug output of mqtt
#define DBG_MQTT 0
// wifi/mqtt credentials
#define WIFI_SSID "WifiName"
#define WIFI_PASSWORD "WifiPassword"
#define MQTT_SERVER "172.20.10.2"
// mqtt client name of tile (1 and 2 in use), also used in topic
#define MQTT_CLIENT_NAME "LOST_CUBE_1"
// amount of bytes that get sent to python via mqtt for config
#define MQTT_NORMAL_CONFIG_DATA_BYTES 10
// amount of bytes that get sent to python via mqtt for quick config
#define MQTT_QUICK_CONFIG_DATA_BYTES 34
// do not use mqtt connection if set to 0 (used for testing)
#define USE_MQTT 1

// = POSITIONING ============================================================
// after this period has passed, a new calculation for the global position gets performed
#define UPDATE_PERIOD 20
// update limit of tile position age to be used for calculation - 100ms
#define TILE_UPDATE_LIMIT 1000
// show the position of the cube on serial when it gets updated
#define PRINT_CUBE_POSITION 0
// show the heading of the cube on serial when it gets updated
#define PRINT_CUBE_HEADING 0
// show the position-mode (sweeping or crossing position)
#define PUBLISH_MODE_MQTT 1
// publishes the cube's heading in addition to the position
#define PUBLISH_HEADING_MQTT 1

// = I2C ====================================================================
// enable debug output of i2c
#define DBG_I2C 0
// size in bytes of one base station configuration for the tiles
#define CONFIG_SIZE 50
// address offset at which the first tile starts
#define I2C_ADDRESS_OFFSET 10
// index of config tile 0 - top tile
#define CONFIG_TILE_INDEX 0
// transmittion frequency of i2c - should match to tile
#define I2C_FREQUENCY 400000
// 36 bytes position+forward+up vectors + 'u' at beginning
#define I2C_POSITION_UPDATE_SIZE 37

// = OTHERS =================================================================
// number of tiles in the cube
#define NO_TILES 5
// maximum number of possible base stations
#define MAX_BASE_STATIONS 16
// cube is 5x5cm (additional 1.5mm for height of diodes on each side)
#define CUBE_SIZE 0.053
// half size of the cube
#define HALF_CUBE_SIZE (CUBE_SIZE/2)


// = PINS ===================================================================
// pin 37 does not work!!
#define I2C_SDA SDA // 4
#define I2C_SCL SCL // is pin D5, and upper connection of connector
//# order:white, red, yellow, blue, green
#define I2C_COM_REQUEST_PINS D0,D1,D2,D3,D6

//#define I2C_SDA 8
//#define I2C_SCL 9
//#define I2C_COM_REQUEST_PINS 19, 20, 21, 38, 39


// = GLOBAL TYPES ===========================================================
// types for vectors and matrices
typedef float vec3d[3];
typedef float mat3x3[3][3];

// operation modes of the cube
enum EspOperationMode {
  normal,
  config_i2c,
  quick_config_i2c,
  send_config_data,
  send_stop_command
};

// struct for the global cube position and heading
typedef struct _CubeData {
  vec3d position;
  // roll, pitch, yaw can be calculated out of these two vectors
  // we built a python script for this if needed
  vec3d forward_vector;
  vec3d up_vector;
} CubeData;

// struct for the position and heading of a connected tile
typedef struct _TileData {
  // contains the timestamp of the last tile update
  long last_update;
  // contains the received tile position
  vec3d tile_position;
  // corrected positions refering to middle of cube
  vec3d corrected_tile_position;
  // contains the received heading vector that points to the from of the tile
  vec3d forward_vector;
  // contains the received heading vector that points to the upside of the tile
  vec3d up_vector;
  // dirty flag to use in global update
  uint8_t dirty;
  // flag that contains if it was sweeping or crossing
  uint8_t is_crossing;
} TileData;
// ==========================================================================

#endif
