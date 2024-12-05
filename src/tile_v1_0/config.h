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

// includeguard
#ifndef CONFIG_
#define CONFIG_

// - STANDALONE -------------------------------------------
// tile is connected via serial and cable to pc
// with this, configuration and positionioning can be done and sent to pc
#define USE_SERIAL 0
// data gets printed in a format that it can be used for the visualization
#define PRINT_DATA_VISUALIZATION 0

// - CUBE MODE  -------------------------------------------
// disable USE_SERIAL and PRINT_DATA_VISUALIZATION
// set I2C_ADDRESS_OFFSET accordingly and then flash individual tiles

// - GENERAL ----------------------------------------------
// Divides the brightnes of the leds (2 -> 100%/2 = 50% brightness) (3->33%) (4->25%)
#define DARKNESS_FACTOR 100
// it is physically impossible to receive beams farer than 7-8m away from a base station - drop calcualted position
#define DISTANCE_LIMIT 10
// dirty limit+update limit becomes latency!
// amount of last received beams are allowed for positioning calculation -> 5 means if the beam was received in the last 5 rounds (1sec), it gets used for calculating
#define DIRTY_LIMIT 10
// contains the update cycles within which the position data of one sensor is used for the general position and heading -> (UPDATE_LIMIT*20ms) is the time how old the data is allowed to be
// set this to 0 when using sweeping only to get the best update rate for fast movements
// we use all sensor position data calculated within 100ms for crossing (improves update rate because of non ideal 100% beam reception)
#define UPDATE_LIMIT 5
// number of maximal possible photodiodes
#define MAX_PHOTODIODES 4
// number of photodiodes in use (cannot exceed MAX_PHOTODIODES)
#define NO_PHOTODIODES 4
// highest base station number
#define MAX_BASE_STATIONS 16
// time to receive light once for configuration of ts4231 [ms]
#define LIGHT_TIMEOUT 500
// minimal possible difference between two beams
#define BEAM_INDEX_DIFFERENCE 15000

// - DBG --------------------------------------------------
// wait before starting the system until serial connection to pc is established for dbg messages
#define DBG_SERIAL 0
// for more information about pio and configuration of ts4231
#define DBG_SENSORS 0
// print values of crossing calculation
#define DBG_CROSSING 0
// print values of sweeping calculation
#define DBG_SWEEPING 0
// if position of the tile should be printed
#define DBG_POSITION 0
// if heading of the tile should be printed
#define DBG_HEADING 0
// show new configurations that are received
#define DBG_CONFIG 0
// show config file storage debug output
#define DBG_CONFIG_FILE 0
// show debug messages of i2c
#define DBG_I2C 0
// debug calibration 
#define DBG_CALIB 0

// - SERIAL -----------------------------------------------
// size of buffer where config that is received via serial is stored
#define SERIAL_CONFIG_DATA_SIZE 51


// - I2C --------------------------------------------------
// i2c tile address of tile 0 (0-10, 1-11, 2-12, 3-13, 4-14)
#define I2C_ADDRESS_OFFSET 10
// amount of sent data bytes for quick config
#define QUICK_CONFIG_DATA_BYTES 34
// amount of sent data bytes for normal config
#define NORMAL_CONFIG_DATA_BYTES 10
// size of configuration line
#define CONFIG_SIZE 50
// 100000, 400000, or 1000000 with only 90% right reception
#define I2C_FREQUENCY 400000
// size to send to esp32 when tile has new position
#define UPDATE_SIZE 37 


// - BS CONFIGURATION -------------------------------------
// print the loaded positioning data from file
#define SHOW_LOADED_CONFIG_DATA 0
// maxumum number of values in a line of the config file
#define MAX_SUBSTRINGS_CONFIG 12
// configuration logging time of each position for normal configuration
#define CONFIG_LOGGING_TIME 5000
// defines the array size of the positions that are logged -> has to be at least 50 for every second + some spare space if something goes wrong
#define CONFIG_LOGGING_POSITIONS 50
// delay until config file gets written permanently after receiving a new configuration
#define CONFIG_FILE_DELAY 1000


// - LEDS -------------------------------------------------
// leds on top of tile
#define NUM_LEDS 8
// indices of leds on the array are the following: [0,1,6,7,4,5,2,3] (they are wired counterclockwise)
// led blink time in [ms]
#define LED_DELAY_TIME 200


// - INTERNAL ---------------------------------------------
#define DF DARKNESS_FACTOR
// colors of leds
#define COL_WHITE  pixels.Color(255/DF, 255/DF, 255/DF)
#define COL_YELLOW pixels.Color(255/DF,   0/DF, 255/DF)
#define COL_GREEN  pixels.Color(  0/DF, 255/DF,   0/DF)
#define COL_BLUE   pixels.Color(  0/DF,   0/DF, 255/DF)
#define COL_PINK   pixels.Color(255/DF,   0/DF, 255/DF)
#define COL_RED    pixels.Color(255/DF,   0/DF,   0/DF)
#define COL_OFF    pixels.Color(  0/DF,   0/DF,   0/DF)


// - PINS -------------------------------------------------
#define UART_TX  0
#define UART_RX  1
#define SPI_SCK  2
#define SPI_COPI 3
#define SPI_CIPO 4
#define SPI_CSN  5
#define I2C_SDA  6
#define I2C_SCL  7
#define PIN_LEDS 8
#define D_PINS 15,  9, 11, 13
#define E_PINS 16, 10, 12, 14
// pin on which pulse is sent for interrupt on esp
#define FREE_PIN_GPIO 21
#define I2C_INT_TO_ESP 20
#define PIN_USERBUTTON 23

// with final tile:
// D4   D1
//
// D2   D3 (connector between D2 and D3)
// connection
// D1 - GPIO9
// E1 - GPIO10
// D2 - GPIO11
// E2 - GPIO12
// D3 - GPIO13
// E3 - GPIO14
// D4 - GPIO15
// E4 - GPIO16
// for software D4 becomes D0, rest stays the same

// ===========================================================================================================================================
// type defines
#include <stdint.h>

// general vector and matrix type
typedef float vec3d[3];
typedef float mat3x3[3][3];

// struct representing sensor positions
typedef struct _SensorPosition{
  vec3d position;
  uint16_t last_update;
} SensorPosition;

// struct for a received beam
typedef struct _BeamParameter {
  uint8_t dirty_1;
  uint32_t index_1;
  uint8_t dirty_2;
  uint32_t index_2;
  uint8_t is_beam_1;
} BeamParameter;

// data representing the decoded beams on one sensor
typedef struct _SensorData{
  // beams can be received from up to 16 stations
  BeamParameter beams[MAX_BASE_STATIONS];
} SensorData;

// operation mode of the tile
enum TileOperationModes 
{
  normal, 
  config_serial, 
  quick_config_serial, 
  config_i2c, 
  quick_config_i2c
};

// states of normal configuration
enum ConfigurationSteps 
{
  wait_next_position,
  record_data, 
  average_and_send_data,
  wait_config_data
};

#endif
