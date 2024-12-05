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

#include "mmath.h"
#include "bs_calibration.h"
#include <Arduino.h>

// periods expressed using 6MHz
float periods[16] = {119875, 119625, 119125, 118625, 118375, 117875, 117625, 117375, 117125, 116125, 114875, 113875, 113375, 112625, 111625, 110875};

float to_rad(float degree)
{
  return degree / 180 * PI;
}

float to_degree(float rad)
{
  return rad / PI * 180;
}

void print_vector(vec3d vector)
{
  Serial.print("[");
  Serial.print(vector[0]);
  Serial.print(",");
  Serial.print(vector[1]);
  Serial.print(",");
  Serial.print(vector[2]);
  Serial.println("]");
}

void print_vector1000(vec3d vector)
{
  Serial.print("[");
  Serial.print(vector[0]*1000.0);
  Serial.print(",");
  Serial.print(vector[1]*1000.0);
  Serial.print(",");
  Serial.print(vector[2]*1000.0);
  Serial.println("]");
}

void print_csv(vec3d vector)
{
  Serial.print(vector[0]);
  Serial.print("\t");
  Serial.print(vector[1]);
  Serial.print("\t");
  Serial.println(vector[2]);
}


// cross product
void cross_product(vec3d a, vec3d b, vec3d result)
{
  result[0] = a[1]*b[2] - a[2]*b[1];
  result[1] = a[2]*b[0] - a[0]*b[2];
  result[2] = a[0]*b[1] - a[1]*b[0];
}


float dot_product(vec3d a, vec3d b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

float vector_length(vec3d a) {
    float pow = dot_product(a, a);
    return sqrt(pow);
}

void vector_divide(vec3d a, float divisor, vec3d result)
{
  result[0] = a[0] / divisor;
  result[1] = a[1] / divisor;
  result[2] = a[2] / divisor;
}

void vector_multiply(vec3d a, float factor, vec3d result)
{
  result[0] = a[0] * factor;
  result[1] = a[1] * factor;
  result[2] = a[2] * factor;
}

void vector_add(vec3d a, vec3d b, vec3d result)
{
  result[0] = a[0] + b[0];
  result[1] = a[1] + b[1];
  result[2] = a[2] + b[2];
}

void vector_subtract(vec3d a, vec3d b, vec3d result)
{
  result[0] = a[0] - b[0];
  result[1] = a[1] - b[1];
  result[2] = a[2] - b[2];
}

// copies b to a
void vector_copy(vec3d dest, vec3d src)
{
  dest[0] = src[0];
  dest[1] = src[1];
  dest[2] = src[2];
}

void matrix_vector_multiply(mat3x3 m, vec3d a, vec3d result)
{
  result[0] = m[0][0] * a[0] + m[0][1] * a[1] + m[0][2] * a[2];
  result[1] = m[1][0] * a[0] + m[1][1] * a[1] + m[1][2] * a[2];
  result[2] = m[2][0] * a[0] + m[2][1] * a[1] + m[2][2] * a[2];
}

bool invert_mat3x3(mat3x3 m, mat3x3 minv)
{
  // computes the inverse of a matrix m
  float determinant = m[0][0] * (m[1][1] * m[2][2] - m[2][1] * m[1][2]) -
                      m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                      m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

  if(abs(determinant) < 1.0E-9)
  {
    Serial.print("[ERROR] matrix singular, determinant: ");
    Serial.println(determinant*1000000000.0);
    // TODO: maybe make value smaller, check with code
    return false;
  }

  float inverse_determinant = 1 / determinant;

  minv[0][0] = (m[1][1] * m[2][2] - m[2][1] * m[1][2]) * inverse_determinant;
  minv[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * inverse_determinant;
  minv[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inverse_determinant;
  minv[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * inverse_determinant;
  minv[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inverse_determinant;
  minv[1][2] = (m[1][0] * m[0][2] - m[0][0] * m[1][2]) * inverse_determinant;
  minv[2][0] = (m[1][0] * m[2][1] - m[2][0] * m[1][1]) * inverse_determinant;
  minv[2][1] = (m[2][0] * m[0][1] - m[0][0] * m[2][1]) * inverse_determinant;
  minv[2][2] = (m[0][0] * m[1][1] - m[1][0] * m[0][1]) * inverse_determinant;
  // singular matrix
  return true;
}

void normalize_vector(vec3d vector, vec3d normalized_vector)
{
  float length = vector_length(vector);
  // normalize
  vector_divide(vector, length, normalized_vector);
  return;
}

void print_mat3x3(mat3x3 matrix)
{
  Serial.print("{");
  Serial.print(matrix[0][0]);
  Serial.print(",");
  Serial.print(matrix[0][1]);
  Serial.print(",");
  Serial.print(matrix[0][2]);
  Serial.println("}");
  Serial.print("{");
  Serial.print(matrix[1][0]);
  Serial.print(",");
  Serial.print(matrix[1][1]);
  Serial.print(",");
  Serial.print(matrix[1][2]);
  Serial.println("}");
  Serial.print("{");
  Serial.print(matrix[2][0]);
  Serial.print(",");
  Serial.print(matrix[2][1]);
  Serial.print(",");
  Serial.print(matrix[2][2]);
  Serial.println("}");
}


// LH2 sends msb first, the pio receiver does it exactly the other way 
// (MSB is in last bit therefore reverse the bits)
uint32_t reverse_bits(uint32_t in)
{
  uint32_t reversed = 0;
  for(int i = 0; i < 32; ++i)
  {
    reversed |= ((in & 0x00000001) << (31 - i));
    in = (in >> 1);
  }
  return reversed;
}


// get the normalized direction vector out of the two angles
void get_normalized_ray(float azimuth, float elevation, vec3d ray)
{
  // my way https://math.stackexchange.com/questions/1150232/finding-the-unit-direction-vector-given-azimuth-and-elevation
  vec3d ray1 = {cos(azimuth) * cos(elevation), sin(azimuth) * cos(elevation), sin(elevation)};
  float length1 = vector_length(ray1);

  // normalize ray length
  vector_divide(ray1, length1, ray);
  return;

}


bool calculate_azimuth_elevation(uint8_t station_number, uint32_t index_1, uint32_t index_2, float *azimuth, float *elevation)
{
  uint32_t tmp_index;

  // index 1 needs to be the smaller one
  if(index_1 > index_2)
  {
    tmp_index = index_1;
    index_1 = index_2;
    index_2 = tmp_index;
  }

   // TOOD imporvement: check upper limit of base station (when sensor is on very top of station) (indexes are nearest to each other at the middle top of the station) (may be addapted to 10k or 15k)
  if((index_2 - index_1) < 20000)
  {
    #if ((DBG_CROSSING == 1) || (DBG_SWEEPING == 1))
      Serial.print("[DBG_CROSSING | DBG_SWEEPING] indexes too close together!");
      Serial.print(" idx1: ");
      Serial.print(index_1);
      Serial.print(" idx2: ");
      Serial.println(index_2);
    #endif
    return false;
  }

  float raw_angle_index_unshifted_1, raw_angle_index_unshifted_2;
  float corrected_angle_index_unshifted_1, corrected_angle_index_unshifted_2;

  float raw_angle_index_1_cf, raw_angle_index_2_cf;
  float corrected_angle_index_1_cf, corrected_angle_index_2_cf;

  raw_angle_index_unshifted_1 = ((index_1 * 8.0) / periods[station_number]) * 2.0 * PI;
  raw_angle_index_unshifted_2 = ((index_2 * 8.0) / periods[station_number]) * 2.0 * PI;
  
  // transform to cf system angles
  raw_angle_index_1_cf = raw_angle_index_unshifted_1 - ((2.0 * PI)/3.0);
  raw_angle_index_2_cf = raw_angle_index_unshifted_2 - ((4.0 * PI)/3.0);
  // use correction based on cf system
  correct_angles(&raw_angle_index_1_cf, &raw_angle_index_2_cf, &corrected_angle_index_1_cf, &corrected_angle_index_2_cf, station_number);

  // transform corrected angles back to our system
  corrected_angle_index_unshifted_1 = corrected_angle_index_1_cf + ((2.0 * PI)/3.0);
  corrected_angle_index_unshifted_2 = corrected_angle_index_2_cf + ((4.0 * PI)/3.0);

  // should be the angles for correction
  float angle_index_1, angle_index_2, b, p;

  /*
  // without calibration correction
  angle_index_1 = ((index_1 * 8.0) / periods[station_number]) * 2 * PI;
  angle_index_2 = ((index_2 * 8.0) / periods[station_number]) * 2 * PI;
  */
  // with calibration correction
  angle_index_1 = corrected_angle_index_unshifted_1;
  angle_index_2 = corrected_angle_index_unshifted_2;

  // 0 degree is normally the backside -> make 0 degrees as frontside (-PI)
  *azimuth = ((angle_index_1 + angle_index_2) / 2.0) - (PI);

  p = PI / 3.0;
  b = (angle_index_2 - angle_index_1) - ((2.0 / 3.0) * PI);

  *elevation = atan(sin(b / 2) / tan(p / 2));

  return true;
}

