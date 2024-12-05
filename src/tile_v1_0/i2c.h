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

// includeguard
#ifndef I2C_SPI_
#define I2C_SPI_

#include "config.h"
#include <Arduino.h>

// send positioning update to cube via i2c
void i2c_send_positioning_update(vec3d pos, vec3d fw_vector, vec3d u_vector, uint8_t mode);

// i2c callback for receiving
void receive_i2c_data(int count);

// i2c callback for sending
void send_i2c_data();

#endif
