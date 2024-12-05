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

// includeguard
#ifndef MMATH_
#define MMATH_

#include "esp_config.h"

// convert degree to radiant
float to_rad(float degree);

// convert radiant to degree
float to_degree(float rad);

// prints a vector
void print_vector(vec3d vector);

// print vectro scaled by 1000
void print_vector1000(vec3d vector);

// prints a vector in csv format (with tabs as seperator)
void print_csv(vec3d vector);

// set a vector to 0
void init_vector(vec3d vector);

// sets a vector to the desired values
void set_vector(vec3d vector, float a, float b, float c);

// cross product
void cross_product(vec3d a, vec3d b, vec3d result);

// dot product
float dot_product(vec3d a, vec3d b);

// length of vector
float vector_length(vec3d a);

// normalize a vector
void normalize_vector(vec3d vector, vec3d normalized_vector);

// vector division with scalar
void vector_divide(vec3d a, float divisor, vec3d result);

// vector multiplication with scalar
void vector_multiply(vec3d a, float factor, vec3d result);

// vector addition
void vector_add(vec3d a, vec3d b, vec3d result);

// vector subtraction
void vector_subtract(vec3d a, vec3d b, vec3d result);

// copies src to dest
void vector_copy(vec3d dest, vec3d src);

// prints a 3x3 matrix
void print_mat3x3(mat3x3 matrix);

// invert matrix - returns true on success, false on fail
bool invert_mat3x3(mat3x3 m, mat3x3 minv);

// matrix vector multiplication
void matrix_vector_multiply(mat3x3 m, vec3d a, vec3d result);

#endif 