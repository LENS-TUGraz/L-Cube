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
#include <PubSubClient.h>

#include "esp_config.h"
#include "esp_mmath.h"

// contains position and heading of the lost cube
extern CubeData cube_data;
// contains position and heading of the tiles
extern TileData tile_data[NO_TILES];
// mqtt client class
extern PubSubClient mqtt_client;

int m = 0;
// the received vectors are already normalized before sending
void update_global_position()
{
  uint8_t dirty_counter = 0;
  uint8_t dirty_counter_crossing = 0;
  vec3d normalized_up_vector;
  vec3d inv_vector_to_cube_center;
  vec3d added_positions;
  vec3d tmp_added_positions;
  vec3d added_forward_vectors;
  vec3d tmp_added_forward_vectors;
  vec3d added_up_vectors;
  vec3d tmp_added_up_vectors;
  vec3d tmp_calculation_vector;
  vec3d tmp_normalized_vector;
  long timestamp = millis();

  // check each tile for an update within preset limit, so it is useable
  for(uint8_t tile_index = 0; tile_index < NO_TILES; ++tile_index)
  {
    if((timestamp - tile_data[tile_index].last_update) < TILE_UPDATE_LIMIT)
    {
	  if(tile_index == 0)
      {
        // top tile is 5mm more away from center
        vector_multiply(tile_data[tile_index].up_vector, (HALF_CUBE_SIZE + 0.005), inv_vector_to_cube_center);
      }
      else
      {
        vector_multiply(tile_data[tile_index].up_vector, HALF_CUBE_SIZE, inv_vector_to_cube_center);
      }
      vector_subtract(tile_data[tile_index].tile_position, inv_vector_to_cube_center, tile_data[tile_index].corrected_tile_position);
      // check if sweeping or crossing position
      if(tile_data[tile_index].is_crossing == 1)
      {
        dirty_counter_crossing++;
        tile_data[tile_index].dirty = 2;
      }
      else
      {
        tile_data[tile_index].dirty = 1;
      }
      dirty_counter++;
    }
  }

  if(dirty_counter == 0)
  {
    #if (PRINT_CUBE_POSITION == 1)
      Serial.print("No new cube position: ");
      print_vector(cube_data.position);
    #endif
    return;
  }

  // if crossing update is available, we use this, as it is more accurate
  if(dirty_counter_crossing >= 1)
  {
    // merge the positions and headings from crossing
    init_vector(added_positions);
    init_vector(added_forward_vectors);
    init_vector(added_up_vectors);
    for(uint8_t tile_index = 0; tile_index < NO_TILES; ++tile_index)
    {
      if(tile_data[tile_index].dirty == 2)
      {
        vector_add(added_positions, tile_data[tile_index].corrected_tile_position, tmp_added_positions);
        vector_copy(added_positions, tmp_added_positions);

        // rotate forward and up vectors for global alignment
        switch(tile_index)
        {
          case 0:
            // top tile - standard alignment of heading vectors
            vector_add(added_forward_vectors, tile_data[tile_index].forward_vector, tmp_added_forward_vectors);
            vector_copy(added_forward_vectors, tmp_added_forward_vectors);

            vector_add(added_up_vectors, tile_data[tile_index].up_vector, tmp_added_up_vectors);
            vector_copy(added_up_vectors, tmp_added_up_vectors);
            break;

          case 1:
            // front tile - forward vector points up, up vector points forward
            vector_add(added_forward_vectors, tile_data[tile_index].up_vector, tmp_added_forward_vectors);
            vector_copy(added_forward_vectors, tmp_added_forward_vectors);

            vector_add(added_up_vectors, tile_data[tile_index].forward_vector, tmp_added_up_vectors);
            vector_copy(added_up_vectors, tmp_added_up_vectors);
            break;
          case 2:
            // tile on right side - forward vector points up, up vector points to right side -> cross product of forward and up points forward
            cross_product(tile_data[tile_index].forward_vector, tile_data[tile_index].up_vector, tmp_calculation_vector);
            normalize_vector(tmp_calculation_vector, tmp_normalized_vector);

            vector_add(added_forward_vectors, tmp_normalized_vector, tmp_added_forward_vectors);
            vector_copy(added_forward_vectors, tmp_added_forward_vectors);

            vector_add(added_up_vectors, tile_data[tile_index].forward_vector, tmp_added_up_vectors);
            vector_copy(added_up_vectors, tmp_added_up_vectors);
            break;
          case 3:
            // tile on back side - forward vector points up, up vector points to back
            vector_subtract(added_forward_vectors, tile_data[tile_index].up_vector, tmp_added_forward_vectors);
            vector_copy(added_forward_vectors, tmp_added_forward_vectors);

            vector_add(added_up_vectors, tile_data[tile_index].forward_vector, tmp_added_up_vectors);
            vector_copy(added_up_vectors, tmp_added_up_vectors);
            break;
            break;
          case 4:
            // left tile - forward vector points up, up vector points to left side -> cross product of up and forward points forward
            cross_product(tile_data[tile_index].up_vector, tile_data[tile_index].forward_vector, tmp_calculation_vector);
            normalize_vector(tmp_calculation_vector, tmp_normalized_vector);

            vector_add(added_forward_vectors, tmp_normalized_vector, tmp_added_forward_vectors);
            vector_copy(added_forward_vectors, tmp_added_forward_vectors);

            vector_add(added_up_vectors, tile_data[tile_index].forward_vector, tmp_added_up_vectors);
            vector_copy(added_up_vectors, tmp_added_up_vectors);
            break;

        }
        tile_data[tile_index].dirty = 0;
      }
      // reset dirty flags from sweeping tiles too but do not use them
      if(tile_data[tile_index].dirty == 1)
      {
        tile_data[tile_index].dirty = 0;
      }
    }

    // average over all data used for calculation and normalize headings
    vector_divide(added_positions, dirty_counter_crossing, tmp_added_positions);
    vector_copy(cube_data.position, tmp_added_positions);

    vector_divide(added_forward_vectors, dirty_counter_crossing, tmp_added_forward_vectors);
    normalize_vector(tmp_added_forward_vectors, cube_data.forward_vector);

    vector_divide(added_up_vectors, dirty_counter_crossing, tmp_added_up_vectors);
    normalize_vector(tmp_added_up_vectors, cube_data.up_vector);
  }
  else
  {
    // merge the positions and headings from sweeping
    init_vector(added_positions);
    init_vector(added_forward_vectors);
    init_vector(added_up_vectors);
    for(uint8_t tile_index = 0; tile_index < NO_TILES; ++tile_index)
    {
      if(tile_data[tile_index].dirty == 1)
      {
        vector_add(added_positions, tile_data[tile_index].corrected_tile_position, tmp_added_positions);
        vector_copy(added_positions, tmp_added_positions);

        // rotate forward and up vectors for global alignment
        switch(tile_index)
        {
          case 0:
            // top tile - standard alignment of heading vectors
            vector_add(added_forward_vectors, tile_data[tile_index].forward_vector, tmp_added_forward_vectors);
            vector_copy(added_forward_vectors, tmp_added_forward_vectors);

            vector_add(added_up_vectors, tile_data[tile_index].up_vector, tmp_added_up_vectors);
            vector_copy(added_up_vectors, tmp_added_up_vectors);
            break;

          case 1:
            // front tile - forward vector points up, up vector points forward
            vector_add(added_forward_vectors, tile_data[tile_index].up_vector, tmp_added_forward_vectors);
            vector_copy(added_forward_vectors, tmp_added_forward_vectors);

            vector_add(added_up_vectors, tile_data[tile_index].forward_vector, tmp_added_up_vectors);
            vector_copy(added_up_vectors, tmp_added_up_vectors);
            break;
          case 2:
            // tile on right side - forward vector points up, up vector points to right side -> cross product of forward and up points forward
            cross_product(tile_data[tile_index].forward_vector, tile_data[tile_index].up_vector, tmp_calculation_vector);
            normalize_vector(tmp_calculation_vector, tmp_normalized_vector);

            vector_add(added_forward_vectors, tmp_normalized_vector, tmp_added_forward_vectors);
            vector_copy(added_forward_vectors, tmp_added_forward_vectors);

            vector_add(added_up_vectors, tile_data[tile_index].forward_vector, tmp_added_up_vectors);
            vector_copy(added_up_vectors, tmp_added_up_vectors);
            break;
          case 3:
            // tile on back side - forward vector points up, up vector points to back
            vector_subtract(added_forward_vectors, tile_data[tile_index].up_vector, tmp_added_forward_vectors);
            vector_copy(added_forward_vectors, tmp_added_forward_vectors);

            vector_add(added_up_vectors, tile_data[tile_index].forward_vector, tmp_added_up_vectors);
            vector_copy(added_up_vectors, tmp_added_up_vectors);
            break;
            break;
          case 4:
            // left tile - forward vector points up, up vector points to left side -> cross product of up and forward points forward
            cross_product(tile_data[tile_index].up_vector, tile_data[tile_index].forward_vector, tmp_calculation_vector);
            normalize_vector(tmp_calculation_vector, tmp_normalized_vector);

            vector_add(added_forward_vectors, tmp_normalized_vector, tmp_added_forward_vectors);
            vector_copy(added_forward_vectors, tmp_added_forward_vectors);

            vector_add(added_up_vectors, tile_data[tile_index].forward_vector, tmp_added_up_vectors);
            vector_copy(added_up_vectors, tmp_added_up_vectors);
            break;

        }
        tile_data[tile_index].dirty = 0;
      }
    }

    // average over all data used for calculation and normalize headings
    vector_divide(added_positions, dirty_counter, tmp_added_positions);
    vector_copy(cube_data.position, tmp_added_positions);

    vector_divide(added_forward_vectors, dirty_counter, tmp_added_forward_vectors);
    normalize_vector(tmp_added_forward_vectors, cube_data.forward_vector);

    vector_divide(added_up_vectors, dirty_counter, tmp_added_up_vectors);
    normalize_vector(tmp_added_up_vectors, cube_data.up_vector);
  }
  

  #if (PRINT_CUBE_POSITION == 1)
    Serial.print("Cube position: ");
    if(dirty_counter_crossing >= 1)
    {
      Serial.print("[c]");
    }
    else
    {
      Serial.print("[s]");
    }
    print_vector(cube_data.position);
  #endif

  #if (PRINT_CUBE_HEADING == 1)
    Serial.print("Cube forward vector: ");
    print_vector(cube_data.forward_vector);
    Serial.print("Cube up vector: ");
    print_vector(cube_data.up_vector);
  #endif

  #if (USE_MQTT == 1)
    // transfer data to pc via MQTT
    mqtt_client.publish(MQTT_CLIENT_NAME"/cube_position", (uint8_t*)cube_data.position, 12);
    
    #if (PUBLISH_MODE_MQTT == 1)
      if(dirty_counter_crossing >= 1)
      {
        mqtt_client.publish(MQTT_CLIENT_NAME"/cube_mode", "c", 1);
      }
      else
      {
        mqtt_client.publish(MQTT_CLIENT_NAME"/cube_mode", "s", 1);
      }
    #endif

    #if (PUBLISH_HEADING_MQTT == 1)
      mqtt_client.publish(MQTT_CLIENT_NAME"/cube_forward_vector", (uint8_t*)cube_data.forward_vector, 12);
      mqtt_client.publish(MQTT_CLIENT_NAME"/cube_up_vector", (uint8_t*)cube_data.up_vector, 12);
    #endif
  #endif

  #if (SERIAL_MODE == 1)
  if(m == 0)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(D10, HIGH);
    m = 1;
  }
  else
  {
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(D10, LOW);
    m = 0;
  }
    
    // write global data
    if(dirty_counter_crossing >= 1)
    {
      // h for crossing
      Serial.print("h");
    }
    else
    {
      // g for sweeping
      Serial.print("g");
    }
    Serial.write((byte*)cube_data.position, 12);
    Serial.write((byte*)cube_data.forward_vector, 12);
    Serial.write((byte*)cube_data.up_vector, 12);
    Serial.println("");

    // could also send data to pc via mqtt, but cable is more reliable
    // mqtt_client.publish("g", cube_data, TEST_DATA_SIZE);
  #endif
}


/*
// positioning test with pseudo data
void perform_calc_pseudo_data()
{
  long fake_timestamp = 0; 

  // test vector for cube at 1,1,1; front is facing x direction, top is facing z direction
  for(int i = 0; i < 5; ++i)
  {
    set_vector(tile_data[i].tile_position,1,1,1);
    set_vector(tile_data[i].forward_vector,0,0,1);
  }
  set_vector(tile_data[0].forward_vector,1,0,0);
  
  set_vector(tile_data[0].up_vector,0,0,1);
  set_vector(tile_data[1].up_vector,1,0,0);
  set_vector(tile_data[2].up_vector,0,-1,0);
  set_vector(tile_data[3].up_vector,-1,0,0);
  set_vector(tile_data[4].up_vector,0,1,0);

  while(1)
  {
    // set timestamp so all data is used in calculation
    fake_timestamp = millis();
    for(int i = 0; i < 5; ++i)
    {
      tile_data[i].last_update = fake_timestamp;
    }
    long past = millis();
    // calculation takes approximately 1ms
    update_global_position();
    Serial.println(millis() - past);
    delay(5000);
  }
}
*/

