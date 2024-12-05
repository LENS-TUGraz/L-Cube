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

// calculation of intersection for one sensor (or nearest point beween two rays)

#include <math.h>
#include "crossing.h"
#include "config.h"
#include "mmath.h"

// headings of base stations
extern vec3d station_positions[MAX_BASE_STATIONS];
extern mat3x3 station_orientations[MAX_BASE_STATIONS];

// min_distance between two lines
bool get_intersection(vec3d station1, vec3d ray1, vec3d station2, vec3d ray2, vec3d intersection, float* min_distance)
{
  // Algorithm: http://geoalgorithms.com/a07-_distance.html#Distance-between-Lines
  vec3d distance_base_stations = {0,0,0};
  vector_subtract(station1, station2, distance_base_stations);

  float a, b, c, d, e;
  a = dot_product(ray1, ray1);
  b = dot_product(ray1, ray2);
  c = dot_product(ray2, ray2);
  d = dot_product(ray1, distance_base_stations);
  e = dot_product(ray2, distance_base_stations);

  float denom = a * c - b * b;
  if (abs(denom) < 1e-5f) {
    return false;
  }

  // Closest point to 2nd line on 1st line
  float t1 = (b * e - c * d) / denom;
  vec3d pt1 = {0,0,0};

  vector_multiply(ray1, t1, pt1);
  vector_add(pt1, station1, pt1);

  // Closest point to 1st line on 2nd line
  float t2 = (a * e - b * d) / denom;
  vec3d pt2 = {0,0,0};
  vector_multiply(ray2, t2, pt2);
  vector_add(pt2, station2, pt2);

  // Result is in the middle
  vec3d tmp = {0,0,0};
  vector_add(pt1, pt2, tmp);
  vector_multiply(tmp, 0.5f, intersection);

  // distance between pt1 and pt2 (minimal distance between rays)
  vector_subtract(pt1, pt2, tmp);
  *min_distance = vector_length(tmp);

  // sanity check to be in range of base station
  if((t1 > DISTANCE_LIMIT) || (t2 > DISTANCE_LIMIT))
  {
    #if (DBG_CROSSING == 1)
      Serial.println("[DBG_CROSSING] calculated crossing point is physically impossible as it is out of reachable area where beams can be decoded");
      Serial.print("[DBG_CROSSING] distance to first base station: ");
      Serial.print(t1);
      Serial.print(" , distance to second base station: ");
      Serial.println(t2);
    #endif
    return false;
  }

  return true;
}


int perform_crossing(uint8_t station_index_1, uint32_t lfsr_index_1_1, uint32_t lfsr_index_1_2, uint8_t station_index_2, uint32_t lfsr_index_2_1, uint32_t lfsr_index_2_2, vec3d sensor_position)
{
  float azimuth_1, elevation_1, azimuth_2, elevation_2;

  // get normalized ray directions from stations
  vec3d ray1 = {0,0,0};
  vec3d ray2 = {0,0,0};

  // rotation with base station orientation
  vec3d rotated_ray1 = {0,0,0};
  vec3d rotated_ray2 = {0,0,0};

  // intersection of the two rays
  vec3d intersection = {0,0,0};
  // distance between the two rays
  float distance = 0;

  #if (DBG_CROSSING == 1)
    Serial.print("[DBG_CROSSING] with station_index_1: ");
    Serial.print(station_index_1);
    Serial.print(" index1: ");
    Serial.print(lfsr_index_1_1);
    Serial.print(" index2: ");
    Serial.print(lfsr_index_1_2);
    Serial.print(" station_index_2: ");
    Serial.print(station_index_2);
    Serial.print(" index1: ");
    Serial.print(lfsr_index_2_1);
    Serial.print(" index2: ");
    Serial.println(lfsr_index_2_2);
  #endif

  if(!calculate_azimuth_elevation(station_index_1, lfsr_index_1_1, lfsr_index_1_2, &azimuth_1, &elevation_1))
  {
    #if (DBG_CROSSING == 1)
      Serial.println("[DBG_CROSSING] angle for station 1 could not be calculated");
    #endif
    return -1;
  }
  
  if(!calculate_azimuth_elevation(station_index_2, lfsr_index_2_1, lfsr_index_2_2, &azimuth_2, &elevation_2))
  {
    #if (DBG_CROSSING == 1)
      Serial.println("[DBG_CROSSING] angle for station 2 could not be calculated");
    #endif
    return -1;
  }

  #if (DBG_CROSSING == 1)
    Serial.print("[DBG_CROSSING] Angles for intersection: a1: ");
    Serial.print(to_degree(azimuth_1));
    Serial.print(" e1: ");
    Serial.print(to_degree(elevation_1));
    Serial.print(" a2: ");
    Serial.print(to_degree(azimuth_2));
    Serial.print(" e2: ");
    Serial.println(to_degree(elevation_2));
  #endif

  // normalized ray vector that points away from base station
  get_normalized_ray(azimuth_1, elevation_1, ray1);
  get_normalized_ray(azimuth_2, elevation_2, ray2);

  // rotate direction vector with orientation of the base station
  matrix_vector_multiply(station_orientations[station_index_1], ray1, rotated_ray1);
  matrix_vector_multiply(station_orientations[station_index_2], ray2, rotated_ray2);

  #if (DBG_CROSSING == 1)
    Serial.println("[DBG_CROSSING] Ray vectors:");
    print_vector(ray1);
    print_vector(ray2);
  #endif

  if(get_intersection(station_positions[station_index_1], rotated_ray1, station_positions[station_index_2], rotated_ray2, intersection, &distance))
  {
    vector_copy(sensor_position, intersection);

    #if (DBG_CROSSING == 1)
      Serial.println("[DBG_CROSSING] got an intersection:");
      print_vector(intersection);
      Serial.print("[DBG_CROSSING] distance between crossing rays: ")
      Serial.println(distance);
      Serial.println();
    #endif
  }
  else
  {
    #if (DBG_CROSSING == 1)
      Serial.println("[DBG_CROSSING] [ERROR] no intersection possible (denominator of algorithm too small");
    #endif
    return -1;
  }
  return 0;
}
