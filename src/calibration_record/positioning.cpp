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

#include "positioning.h"
#include <Arduino.h>
#include "config.h"
#include "mmath.h"
#include "sweeping.h"


// data representing the decoded beams recevied by the sensors
extern SensorData sensor_data[NO_PHOTODIODES];
// position of the photodiodes
extern SensorPosition sensor_position[NO_PHOTODIODES];

// position of the middle of the tile
extern vec3d tile_position;
// vector that points from middle to front side of the tile (from usb connector to the point between d4&d1)
extern vec3d forward_vector;
// vector that points 90 degrees to the plane which is constructed by the 4 photodiodes
extern vec3d up_vector;
// vector that points from middle to the left side of the tile
extern vec3d side_vector;

// sweeping calculation was successful
int sweeping_successful = 0;

// temporary result of the sensor positions that are added and then divided by 2 to get tile_position
vec3d added_sensor_positions = {0};
// temp result to normalize forward vector
vec3d tmp_forward_vector = {0};
// temp result to normalize side vector
vec3d tmp_side_vector = {0};
// temp result to normalize up vector
vec3d tmp_up_vector = {0};

// indexes of sensors to use for heading and position calculation
uint8_t index_forward_1 = 0;
uint8_t index_forward_2 = 0;
uint8_t index_side_1 = 0;
uint8_t index_side_2 = 0;
uint8_t index_tile_1 = 0;
uint8_t index_tile_2 = 0;

// global position calculation with the use of three sensor positions - omitted_sensor is the sensor index which is not used in the calculation
void calculate_global_position_and_heading(uint8_t omitted_sensor)
{
  // most probably this case
  if(omitted_sensor == 3)
  {
    index_tile_1 = 1;
    index_tile_2 = 2;
    index_forward_1 = 0;
    index_forward_2 = 2;
    index_side_1 = 0;
    index_side_2 = 1;
  }
  else if(omitted_sensor == 0)
  {
    index_tile_1 = 1;
    index_tile_2 = 2;
    index_forward_1 = 1;
    index_forward_2 = 3;
    index_side_1 = 2;
    index_side_2 = 3;
  }
  else if(omitted_sensor == 1)
  {
    index_tile_1 = 0;
    index_tile_2 = 3;
    index_forward_1 = 0;
    index_forward_2 = 2;
    index_side_1 = 2;
    index_side_2 = 3;
  }
  //omitted_sensor == 2
  else
  {
    index_tile_1 = 0;
    index_tile_2 = 3;
    index_forward_1 = 1;
    index_forward_2 = 3;
    index_side_1 = 0;
    index_side_2 = 1;
  }

  // position of the middle of the tile
  vector_add(sensor_position[index_tile_1].position, sensor_position[index_tile_2].position, added_sensor_positions);
  vector_divide(added_sensor_positions, 2.0, tile_position);

  // heading of the tile
  vector_subtract(sensor_position[index_forward_1].position, sensor_position[index_forward_2].position, tmp_forward_vector);
  vector_subtract(sensor_position[index_side_1].position, sensor_position[index_side_2].position, tmp_side_vector);
  cross_product(tmp_forward_vector, tmp_side_vector, tmp_up_vector);

  // normalized heading vectors
  normalize_vector(tmp_forward_vector, forward_vector);
  normalize_vector(tmp_side_vector, side_vector);
  normalize_vector(tmp_up_vector, up_vector);

  #if (DBG_POSITION == 1)
    Serial.print("[DBG_POSITION] ");
    print_vector(tile_position);
  #endif

  #if (DBG_HEADING == 1)
    Serial.print("[DBG_HEADING] forward:");
    print_vector(normalized_forward_vector);
    Serial.print("[DBG_HEADING] up:");
    print_vector(normalized_up_vector);
  #endif
}


void perform_positioning()
{
  // -----------------------------------------------------------------------
  // - JUST A DEMO FUNCTION HERE TO CHECK CORRECTION OF ANGLES IF REQUIRED -
  // -----------------------------------------------------------------------
  uint8_t bs_index = 0;
  // sensors 0,1,2
  sweeping_successful = perform_sweeping(bs_index, 0, sensor_data[0].beams[bs_index].index_1, sensor_data[0].beams[bs_index].index_2, 1, sensor_data[1].beams[bs_index].index_1, sensor_data[1].beams[bs_index].index_2, 
  2, sensor_data[2].beams[bs_index].index_1, sensor_data[2].beams[bs_index].index_2, sensor_position[0].position, sensor_position[1].position, sensor_position[2].position);
        
  // heading and position calculation
  calculate_global_position_and_heading(3);

  reset_dirty_bits();
}


void reset_dirty_bits()
{
  // reset all dirty bits after one cycle (TODO improvement, maybe we want to use older ones too)
  for(int sensor_index = 0; sensor_index < NO_PHOTODIODES; ++sensor_index)
  {
    for(int bs_index = 0; bs_index < MAX_BASE_STATIONS; ++bs_index)
    {
      sensor_data[sensor_index].beams[bs_index].dirty_1 = 0;
      sensor_data[sensor_index].beams[bs_index].dirty_2 = 0;
    }
  }
}
