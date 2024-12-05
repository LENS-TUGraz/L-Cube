/*
 * L-Cube Calibration Record
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
// used for sending recorded calibration to pc
#define USE_SERIAL 1

// - GENERAL ----------------------------------------------
// Divides the brightnes of the leds (2 -> 100%/2 = 50% brightness) (3->33%) (4->25%)
#define DARKNESS_FACTOR 100
// it is physically impossible to receive beams farer than 7-8m away from a base station - drop calcualted position
#define DISTANCE_LIMIT 10
// number of maximal possible photodiodes
#define MAX_PHOTODIODES 4
// number of photodiodes in use (cannot exceed MAX_PHOTODIODES)
#define NO_PHOTODIODES 4
// highest base station number
#define MAX_BASE_STATIONS 16
// time to receive light once for configuration of ts4231 [ms]
#define LIGHT_TIMEOUT 500

// - DBG --------------------------------------------------
// wait before starting the system until serial connection to pc is established for dbg messages
#define DBG_SERIAL 1
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
// debug calibration 
#define DBG_CALIB 1


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
#define SPI_INT_TO_ESP 21
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
  uint32_t last_update;
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
  config_spi, 
  quick_config_spi
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
