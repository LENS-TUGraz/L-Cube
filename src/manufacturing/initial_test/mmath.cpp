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

#include "mmath.h"
#include <Arduino.h>
//#include "config.h"

// we could also express this with 6MHz clock (everything /8) and do not multiply with 8 in calculation, as we only need this for conversion
// expressed using a 48 MHz clock (per bit)
float periods[16] = {959000, 957000, 953000, 949000, 947000, 943000, 941000, 939000, 937000, 929000, 919000, 911000, 907000, 901000, 893000, 887000};

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

  if((index_2 - index_1) < 20000) // TOOD: check upper limit of base station (may be addapted to 10k or 15k)
  {
    #ifdef VALUES_SWEEPING
      Serial.print("[ERROR] indexes too close together!");
      Serial.print(" idx1: ");
      Serial.print(index_1);
      Serial.print(" idx2: ");
      Serial.println(index_2);
    #endif
    return false;
  }

  float angle_index_1, angle_index_2, b, p;

  angle_index_1 = ((index_1 * 8.0) / periods[station_number]) * 2 * PI;
  angle_index_2 = ((index_2 * 8.0) / periods[station_number]) * 2 * PI;

  // 0 degree is normally the backside -> make 0 degrees as frontside
  *azimuth = ((angle_index_1 + angle_index_2) / 2.0) - (PI);

  p = PI / 3.0;
  b = (angle_index_2 - angle_index_1) - ((2.0 / 3.0) * PI);

  *elevation = atan(sin(b / 2) / tan(p / 2));

  return true;

}

