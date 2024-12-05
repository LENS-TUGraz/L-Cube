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

#include "positioning.h"
#include <Arduino.h>
#include "config.h"
#include "mmath.h"
#include "crossing.h"
#include "sweeping.h"
#include "i2c.h"


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

// counter for sensors that are available for sweeping
uint8_t count_dirty_sensors = 0;
// contains the index of the sensor which cannot be used for sweeping
uint8_t unavailable_sensor_index = 3;
// sweeping calculation was successful
int sweeping_successful = 0;
// flag to know if a new position is calculated
uint8_t new_position = 0;
// first candidate for crossing calculation
int first_crossing_candidate = -1;
// number of outdated sensor positions
uint8_t outdated_positions = 0;
// contains the index of the outdated sensor position, when only one is outdated so calculation is still possible
uint8_t outdated_index = 3;

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

// current position from crossing or sweeping
uint8_t is_sweeping_position = 0;

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
    if(is_sweeping_position == 1)
    {
      // from sweeping algo
      Serial.print("(s) ");
    }
    else
    {
      // from crossing algo
      Serial.print("(c) ");
    }
    print_vector(tile_position);
  #endif
  #if (DBG_HEADING == 1)
    Serial.print("[DBG_HEADING] forward:");
    print_vector(forward_vector);
    Serial.print("[DBG_HEADING] up:");
    print_vector(up_vector);
  #endif

  #if (PRINT_DATA_VISUALIZATION == 1)
    // s for sweeping t for crossing mode of tile position
    if(is_sweeping_position == 1)
    {
      // from sweeping algo
      Serial.print("s");
    }
    else
    {
      // from crossing algo
      Serial.print("t");
    }
    Serial.write((byte*)tile_position, 12);
    Serial.write((byte*)forward_vector, 12);
    Serial.write((byte*)up_vector, 12);
    Serial.println("");
  #endif
}

// returns if the dirty counter (if != 0, contains passed cycles when index was received)
// old version -> dirty needs to be exact 1
int is_index_dirty_ok(uint8_t dirty_bit)
{
  if((dirty_bit != 0) && (dirty_bit <= DIRTY_LIMIT))
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

void perform_positioning()
{
  // normal calculation of position
  // crossing first
  // check for all sensors
  for(int sensor_index = 0; sensor_index < NO_PHOTODIODES; ++sensor_index)
  {
    first_crossing_candidate = -1;
    // we use first possible combination of crossing for each sensor
    for(int bs_index = 0; bs_index < MAX_BASE_STATIONS; ++bs_index)
    {
      if(is_index_dirty_ok(sensor_data[sensor_index].beams[bs_index].dirty_1) && is_index_dirty_ok(sensor_data[sensor_index].beams[bs_index].dirty_2))
      {
        // possible candidate
        if(first_crossing_candidate == -1)
        {
          first_crossing_candidate = bs_index;
        }
        else
        {
          // found pair, do crossing for this sensor
          perform_crossing(first_crossing_candidate, sensor_data[sensor_index].beams[first_crossing_candidate].index_1, sensor_data[sensor_index].beams[first_crossing_candidate].index_2, 
                            bs_index, sensor_data[sensor_index].beams[bs_index].index_1, sensor_data[sensor_index].beams[bs_index].index_2, sensor_position[sensor_index].position);

          sensor_position[sensor_index].last_update = 0;
          break;
        }
      }
    }
  }

  // set flag to know if a new position gets calculated
  new_position = 0;
  // check if enough data for orientation and global position of tile is available
  outdated_positions = 0;
  // if there is data on all 4 photodiodes, we omit photodiode 3
  outdated_index = 3;
  // check if sweeping required
  for(int sensor_index = 0; sensor_index < NO_PHOTODIODES; ++sensor_index)
  {
    if(sensor_position[sensor_index].last_update > UPDATE_LIMIT)
    {
      outdated_index = sensor_index;
      outdated_positions += 1;
    }
  }

  // sweeping required
  if(outdated_positions > 1)
  {
    // check all base stations for sweeping, abort if first sweep is successful
    for(int bs_index = 0; bs_index < MAX_BASE_STATIONS; ++bs_index)
    {
      count_dirty_sensors = 0;
      unavailable_sensor_index = 3;
      for(int sensor_index = 0; sensor_index < NO_PHOTODIODES; ++sensor_index)
      {
        // both indexes need to be new
        if(is_index_dirty_ok(sensor_data[sensor_index].beams[bs_index].dirty_1) && is_index_dirty_ok(sensor_data[sensor_index].beams[bs_index].dirty_2))
        {
          count_dirty_sensors++;
        }
        else
        {
          unavailable_sensor_index = sensor_index;
        }
      }
      // sweeping possible
      if(count_dirty_sensors >= 3)
      {
        sweeping_successful = -1;
        // check which beams to use
        if(unavailable_sensor_index == 3)
        {
          // sensors 0,1,2
          sweeping_successful = perform_sweeping(bs_index, 0, sensor_data[0].beams[bs_index].index_1, sensor_data[0].beams[bs_index].index_2, 1, sensor_data[1].beams[bs_index].index_1, sensor_data[1].beams[bs_index].index_2, 
          2, sensor_data[2].beams[bs_index].index_1, sensor_data[2].beams[bs_index].index_2, sensor_position[0].position, sensor_position[1].position, sensor_position[2].position);
        }
        else if(unavailable_sensor_index == 0)
        {
          // sensors 1,2,3
          sweeping_successful = perform_sweeping(bs_index, 1, sensor_data[1].beams[bs_index].index_1, sensor_data[1].beams[bs_index].index_2, 2, sensor_data[2].beams[bs_index].index_1, sensor_data[2].beams[bs_index].index_2, 
          3, sensor_data[3].beams[bs_index].index_1, sensor_data[3].beams[bs_index].index_2, sensor_position[1].position, sensor_position[2].position, sensor_position[3].position);
        }
        else if(unavailable_sensor_index == 1)
        {
          // sensors 0,2,3
          sweeping_successful = perform_sweeping(bs_index, 0, sensor_data[0].beams[bs_index].index_1, sensor_data[0].beams[bs_index].index_2, 2, sensor_data[2].beams[bs_index].index_1, sensor_data[2].beams[bs_index].index_2, 
          3, sensor_data[3].beams[bs_index].index_1, sensor_data[3].beams[bs_index].index_2, sensor_position[0].position, sensor_position[2].position, sensor_position[3].position);
        }
        else
        {
          // unavailable_sensor_index == 2
          // sensors 0,1,3
          sweeping_successful = perform_sweeping(bs_index, 0, sensor_data[0].beams[bs_index].index_1, sensor_data[0].beams[bs_index].index_2, 1, sensor_data[1].beams[bs_index].index_1, sensor_data[1].beams[bs_index].index_2, 
          3, sensor_data[3].beams[bs_index].index_1, sensor_data[3].beams[bs_index].index_2, sensor_position[0].position, sensor_position[1].position, sensor_position[3].position);
        }
        
        // if sweeping was successful, abort and do final calculation
        if(sweeping_successful == 0)
        {
          // calculate global position and heading
          is_sweeping_position = 1;
          calculate_global_position_and_heading(unavailable_sensor_index);
          new_position = 1;
          break;
        }
      }
    }
  }
  else
  {
    // heading and position calculation
    // crossing position
    is_sweeping_position = 0;
    calculate_global_position_and_heading(outdated_index);
    new_position = 1;
  }

  // new position available?
  if(new_position == 1)
  {
    // TODO improvement: low pass filter could improve positioning!
	
    // send new position data to esp
    i2c_send_positioning_update(tile_position, forward_vector, up_vector, is_sweeping_position);
  }
  else
  {
    #if (DBG_POSITION == 1)
      //Serial.println("[DBG_POSITION] no position update available");
    #endif
  }
  
  // update all dirty bits after one cycle
  update_dirty_bits();

  // update last_updates
  for(int sensor_index = 0; sensor_index < NO_PHOTODIODES; ++sensor_index)
  {
    // max lifetime is 3 days (any number choosen)
    if(sensor_position[sensor_index].last_update < 18000)
    {
      sensor_position[sensor_index].last_update += 1;
    }
  }
}

void update_dirty_bits()
{
  for(int sensor_index = 0; sensor_index < NO_PHOTODIODES; ++sensor_index)
  {
    for(int bs_index = 0; bs_index < MAX_BASE_STATIONS; ++bs_index)
    {
      if(sensor_data[sensor_index].beams[bs_index].dirty_1 != 0)
      {
        // update until it overflows to 0 where it stays 0
        sensor_data[sensor_index].beams[bs_index].dirty_1++;
      }
      if(sensor_data[sensor_index].beams[bs_index].dirty_2 != 0)
      {
        // update until it overflows to 0 where it stays 0
        sensor_data[sensor_index].beams[bs_index].dirty_2++;
      }
    }
  }
}
