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

#include <math.h>
#include <Arduino.h>
#include "sweeping.h"
#include "config.h"
#include "mmath.h"

extern vec3d station_positions[MAX_BASE_STATIONS];
extern mat3x3 station_orientations[MAX_BASE_STATIONS];

uint8_t get_sensor_distances(uint8_t sensor_index_1, uint8_t sensor_index_2, uint8_t sensor_index_3, float *distance_AB, float *distance_BC, float *distance_AC)
{
  // indexes need to be ordered
  if((sensor_index_1 >= sensor_index_2) || (sensor_index_2 >= sensor_index_3))
  {
    Serial.println("[ERROR] indexes not in rising order");
    return 0;
  }

  uint8_t sensor_active[4] = {0};

  sensor_active[sensor_index_1] = 1;
  sensor_active[sensor_index_2] = 1;
  sensor_active[sensor_index_3] = 1;

  if(sensor_active[3] == 0)
  {
    // we only use this for manufacturing test
    *distance_AB = 0.035;
    *distance_BC = 0.0494975;
    *distance_AC = 0.035;

    return 1;
  }
  else
  {
    Serial.println("ERROR");
  }

  return 0;
}

float perform_sweeping_manufacturing(uint8_t station_index, uint8_t sensor_index_1, uint32_t lfsr_index_1_1, uint32_t lfsr_index_1_2, uint8_t sensor_index_2, uint32_t lfsr_index_2_1, uint32_t lfsr_index_2_2, uint8_t sensor_index_3, uint32_t lfsr_index_3_1, uint32_t lfsr_index_3_2)
{
  #if (DBG_SWEEPING == 1)
    Serial.print("[SWEEPING] with station_index: ");
    Serial.print(station_index);
    Serial.print(" sensor_index_1: ");
    Serial.print(sensor_index_1);
    Serial.print(" lfsr_index_1_1: ");
    Serial.print(lfsr_index_1_1);
    Serial.print(" lfsr_index_1_2: ");
    Serial.print(lfsr_index_1_2);
    Serial.print(" sensor_index_2: ");
    Serial.print(sensor_index_2);
    Serial.print(" lfsr_index_2_1: ");
    Serial.print(lfsr_index_2_1);
    Serial.print(" lfsr_index_2_2: ");
    Serial.print(lfsr_index_2_2);
    Serial.print(" sensor_index_3: ");
    Serial.print(sensor_index_3);
    Serial.print(" lfsr_index_3_1: ");
    Serial.print(lfsr_index_3_1);
    Serial.print(" lfsr_index_3_2: ");
    Serial.println(lfsr_index_3_2);
    Serial.println("---");
  #endif
  // get azimuth and elevation angles
  float azimuth_1, elevation_1, azimuth_2, elevation_2, azimuth_3, elevation_3;

  if(!calculate_azimuth_elevation(station_index, lfsr_index_1_1, lfsr_index_1_2, &azimuth_1, &elevation_1))
  {
    #if (DBG_SWEEPING == 1)
      Serial.println("[ERROR] angle for sensor 1 could not be calculated");
    #endif
    return -1;
  }
  
  if(!calculate_azimuth_elevation(station_index, lfsr_index_2_1, lfsr_index_2_2, &azimuth_2, &elevation_2))
  {
    #if (DBG_SWEEPING == 1)
      Serial.println("[ERROR] angle for sensor 2 could not be calculated");
    #endif
    return -1;
  }

  if(!calculate_azimuth_elevation(station_index, lfsr_index_3_1, lfsr_index_3_2, &azimuth_3, &elevation_3))
  {
    #if (DBG_SWEEPING == 1)
      Serial.println("[ERROR] angle for sensor 3 could not be calculated");
    #endif
    return -1;
  }

  #if (DBG_SWEEPING == 1)
    Serial.print("Angles for sweeping: a1: ");
    Serial.print(to_degree(azimuth_1));
    Serial.print(" e1: ");
    Serial.print(to_degree(elevation_1));
    Serial.print(" a2: ");
    Serial.print(to_degree(azimuth_2));
    Serial.print(" e2: ");
    Serial.print(to_degree(elevation_2));
    Serial.print(" a3: ");
    Serial.print(to_degree(azimuth_3));
    Serial.print(" e3: ");
    Serial.println(to_degree(elevation_3));
  #endif

  // get normalized ray directions from station to sensor (to calculate coordinates later)
  vec3d ray1 = {0,0,0};
  vec3d ray2 = {0,0,0};
  vec3d ray3 = {0,0,0};
  get_normalized_ray(azimuth_1, elevation_1, ray1);
  get_normalized_ray(azimuth_2, elevation_2, ray2);
  get_normalized_ray(azimuth_3, elevation_3, ray3);

  #if (DBG_SWEEPING == 1)
    Serial.println("rays:");
    print_vector1000(ray1);
    print_vector1000(ray2);
    print_vector1000(ray3);
  #endif

  float ca_ab = dot_product(ray1, ray2);
  float ca_bc = dot_product(ray2, ray3);
  float ca_ac = dot_product(ray3, ray1);

  #if (DBG_SWEEPING == 1)
    Serial.print("Angles between rays: ab: ");
    Serial.print(to_degree(acos(ca_ab)));
    Serial.print(" bc: ");
    Serial.print(to_degree(acos(ca_bc)));
    Serial.print(" ac: ");
    Serial.println(to_degree(acos(ca_ac)));
  #endif

  float distance_AB, distance_BC, distance_AC;

  if(!get_sensor_distances(sensor_index_1, sensor_index_2, sensor_index_3, &distance_AB, &distance_BC, &distance_AC))
  {
    #if (DBG_SWEEPING == 1)
      Serial.println("[ERROR] distances of sensors could not be calculated");
    #endif
    return -1;
  }

  #if (DBG_SWEEPING == 1)
    Serial.print("distances AB, BC, AC [cm]: ");
    Serial.print(distance_AB*100.0);
    Serial.print("   ");
    Serial.print(distance_BC*100.0);
    Serial.print("   ");
    Serial.println(distance_AC*100.0);
  #endif

  // use newtons root finding method to get position out of equation system
  // random init vals # must not be equal -> singular matrix
  // Ra = 0.3 Rb = 0.2 Rc = 0.1
  vec3d Rk = {0.3, 0.2, 0.1};
  vec3d Rk1 = {0};
  vec3d fRk = {0};
  vec3d intermediate_result = {0};

  mat3x3 jacobian = {0};
  mat3x3 inverse_jacobian = {0};

  // TODO: maybe base this on values of error, 100 should be more than enough,as system already converges at around 50
  // 50 iterations are already ok, so 100 should be more than enough -> takes about 8ms with 100 iterations
  for(int i = 0; i < 100; ++i)
  {
    jacobian[0][0] = 2 * Rk[0] - 2 * Rk[1] * ca_ab;
    jacobian[0][1] = 2 * Rk[1] - 2 * Rk[0] * ca_ab;
    jacobian[0][2] = 0;
    jacobian[1][0] = 0;
    jacobian[1][1] = 2 * Rk[1] - 2 * Rk[2] * ca_bc;
    jacobian[1][2] = 2 * Rk[2] - 2 * Rk[1] * ca_bc;
    jacobian[2][0] = 2 * Rk[0] - 2 * Rk[2] * ca_ac;
    jacobian[2][1] = 0;
    jacobian[2][2] = 2 * Rk[2] - 2 * Rk[0] * ca_ac;

    fRk[0] = Rk[0] * Rk[0] + Rk[1] * Rk[1] - 2 * Rk[0] * Rk[1] * ca_ab - distance_AB * distance_AB;
    fRk[1] = Rk[1] * Rk[1] + Rk[2] * Rk[2] - 2 * Rk[1] * Rk[2] * ca_bc - distance_BC * distance_BC;
    fRk[2] = Rk[0] * Rk[0] + Rk[2] * Rk[2] - 2 * Rk[0] * Rk[2] * ca_ac - distance_AC * distance_AC;

    if(!invert_mat3x3(jacobian, inverse_jacobian))
    {
      #if (DBG_SWEEPING == 1)
        Serial.println("[ERROR] jacobian matrix is singular and cannot be inverted");
      #endif
      return -1;
    }

    // Rk1 = Rk - inverse_jacobian*fRk
    matrix_vector_multiply(inverse_jacobian, fRk, intermediate_result);
    vector_subtract(Rk, intermediate_result, Rk1);
    // Rk = Rk_plus1
    vector_copy(Rk, Rk1);
  }
  
  #if (DBG_SWEEPING == 1)
    // checking equation: fRk should be 0 for all 3 dimensions
    Serial.print("fRk should be 0, then position is found, fRk*1000: ");
    print_vector1000(fRk);
    Serial.print("Rk:");
    print_vector(Rk);
  #endif
  

  vec3d sensor_pos1 = {0,0,0};
  vec3d sensor_pos2 = {0,0,0};
  vec3d sensor_pos3 = {0,0,0};

  // get coordinates if base station was origin
  vector_multiply(ray1, Rk[0], sensor_pos1);
  vector_multiply(ray2, Rk[1], sensor_pos2);
  vector_multiply(ray3, Rk[2], sensor_pos3);

  // rotate according to heading of base station
  // TODO!

  // add coordinates of base station
  // TODO! - use base station coordinates and heading which should be transferred via function params

  #if (DBG_SWEEPING == 1)
    Serial.print("Sensor1 position:");
    print_vector(sensor_pos1);
    Serial.print("Sensor2 position:");
    print_vector(sensor_pos2);
    Serial.print("Sensor3 position:");
    print_vector(sensor_pos3);
  #endif

  uint8_t sensor_active[4] = {0};

  sensor_active[sensor_index_1] = 1;
  sensor_active[sensor_index_2] = 1;
  sensor_active[sensor_index_3] = 1;

  vec3d tile_position = {0,0,0};
  vec3d tmp = {0,0,0};

  if(!sensor_active[0])
  {
    vector_add(sensor_pos1, sensor_pos2, tmp);
  }
  if((!sensor_active[1]) || (!sensor_active[2]))
  {
    vector_add(sensor_pos1, sensor_pos3, tmp);
  }
  if(!sensor_active[3])
  {
    vector_add(sensor_pos2, sensor_pos3, tmp);
  }
  vector_divide(tmp, 2.0, tile_position);

  #if (DBG_SWEEPING == 1)
    Serial.println("[SWEEPING] with station as origin:");
    print_vector(tile_position);
  #endif

//  for final algorithm when a configuration is present, we dont need this for manufacturing test
//  matrix_vector_multiply(rota, tile_position, tmp);
//  vector_add(tmp, posi, tile_position);
//  #if (DBG_SWEEPING == 1)
//    Serial.print("[SWEEPING] Tile position (middle of board): ");
//    print_csv(tile_position);
//  #endif

  float distance_to_basestation = (Rk[0] + Rk[2]) / 2.0;
  return distance_to_basestation;
}

