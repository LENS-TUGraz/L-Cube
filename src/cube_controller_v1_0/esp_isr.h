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
#ifndef ESP_ISR_
#define ESP_ISR_

#include <stdint.h>

// isr callbacks i2c com request
void isr_i2c0();
void isr_i2c1();
void isr_i2c2();
void isr_i2c3();
void isr_i2c4();

#endif