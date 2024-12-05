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

// change for actual hardware
#define PROTOTYPE 0

// Divides the brightnes of the leds (2 -> 100%/2 = 50% brightness) (3->33%) (4->25%)
#define DARKNESS_FACTOR 100
#define DF DARKNESS_FACTOR

// wait with test until serial connection to pc is established for dbg messages
#define DBG_SERIAL 0
// for more information about pio and configuration of ts4231
#define DBG_SENSORS 0
// print angles and indexes of sweeping calculation
#define DBG_SWEEPING 0

#define DECODING_CYCLES 3
#define AZIMUTH_ELEVATION_MAX_TRIES 10
// limit in degrees away from 0
#define AZIMUTH_ELEVATION_LIMIT 10
#define TIMEOUT_CYCLES_AZIMUTH_ELEVATION 100
#define SWEEPING_DISTANCE 1
#define SWEEPING_TOLERANCE 0.5
#define TIMEOUT_CYCLES_SWEEPING 100

#if (PROTOTYPE == 1)
  #define PIN_LEDS 17
  #define D_PINS  3, 0, 26, 28
  #define E_PINS  4, 1, 27, 29
#else
  #define PIN_LEDS 8
  #define D_PINS 15,  9, 11, 13
  #define E_PINS 16, 10, 12, 14
#endif

// leds on top of tile
#define NUM_LEDS 8
// time until next led is set on [ms]
#define DELAY_TIME 500

#define COL_WHITE pixels.Color(255/DF, 255/DF, 255/DF)
#define COL_GREEN pixels.Color(  0/DF, 255/DF,   0/DF)
#define COL_BLUE  pixels.Color(  0/DF,   0/DF, 255/DF)
#define COL_PINK  pixels.Color(255/DF,   0/DF, 255/DF)
#define COL_RED   pixels.Color(255/DF,   0/DF,   0/DF)
#define COL_OFF   pixels.Color(  0/DF,   0/DF,   0/DF)

// number of maximal possible photodiodes
#define MAX_PHOTODIODES 4
// number of photodiodes in use (cannot exceed MAX_PHOTODIODES)
#define NO_PHOTODIODES 4

// highest base station number
#define MAX_BASE_STATIONS 16

// time to receive light once for configuration of ts4231 [ms]
#define LIGHT_TIMEOUT 500

// pins
#define UART_TX  0
#define UART_RX  1
#define SPI_SCK  2
#define SPI_COPI 3
#define SPI_CIPO 4
#define SPI_CSN  5
#define I2C_SDA  6
#define I2C_SCL  7
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
