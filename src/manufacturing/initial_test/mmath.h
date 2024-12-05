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

// includeguard
#ifndef MMATH_
#define MMATH_

//#include <Arduino.h>
#include <stdint.h>

typedef float vec3d[3];
typedef float mat3x3[3][3];

// convert degree to radiant
float to_rad(float degree);

// convert radiant to degree
float to_degree(float rad);

// prints a vector
void print_vector(vec3d vector);

// scaled by 1000
void print_vector1000(vec3d vector);

// prints a vector in csv format (with tabs as seperator)
void print_csv(vec3d vector);

// cross product
void cross_product(vec3d a, vec3d b, vec3d result);

float dot_product(vec3d a, vec3d b);

float vector_length(vec3d a);

void vector_divide(vec3d a, float divisor, vec3d result);

void vector_multiply(vec3d a, float factor, vec3d result);

void vector_add(vec3d a, vec3d b, vec3d result);

void vector_subtract(vec3d a, vec3d b, vec3d result);

// copies src to dest
void vector_copy(vec3d dest, vec3d src);

void matrix_vector_multiply(mat3x3 m, vec3d a, vec3d result);

// returns true on success, false on fail
bool invert_mat3x3(mat3x3 m, mat3x3 minv);

// get the normalized direction vector out of the two angles
void get_normalized_ray(float azimuth, float elevation, vec3d ray);

// returns true if calculation was successful
bool calculate_azimuth_elevation(uint8_t station_number, uint32_t index_1, uint32_t index_2, float *azimuth, float *elevation);

#endif 