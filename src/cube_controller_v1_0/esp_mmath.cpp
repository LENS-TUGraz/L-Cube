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
#include "esp_mmath.h"

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


void init_vector(vec3d vector)
{
  vector[0] = 0;
  vector[1] = 0;
  vector[2] = 0;
}


void set_vector(vec3d vector, float a, float b, float c)
{
  vector[0] = a;
  vector[1] = b;
  vector[2] = c;
}


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


void normalize_vector(vec3d vector, vec3d normalized_vector)
{
  float length = vector_length(vector);
  // normalize
  vector_divide(vector, length, normalized_vector);
  return;
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


void vector_copy(vec3d dest, vec3d src)
{
  dest[0] = src[0];
  dest[1] = src[1];
  dest[2] = src[2];
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


void matrix_vector_multiply(mat3x3 m, vec3d a, vec3d result)
{
  result[0] = m[0][0] * a[0] + m[0][1] * a[1] + m[0][2] * a[2];
  result[1] = m[1][0] * a[0] + m[1][1] * a[1] + m[1][2] * a[2];
  result[2] = m[2][0] * a[0] + m[2][1] * a[1] + m[2][2] * a[2];
}

