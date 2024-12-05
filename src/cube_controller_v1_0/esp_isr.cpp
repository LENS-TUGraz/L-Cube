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
#include "esp_isr.h"
#include "esp_config.h"

extern uint8_t i2c_flag[NO_TILES];

// set flag for i2c communication
void isr_i2c0()
{
  if(i2c_flag[0] == 0)
  {
    i2c_flag[0] = 1;
  }
}

void isr_i2c1()
{
  if(i2c_flag[1] == 0)
  {
    i2c_flag[1] = 1;
  }
}

void isr_i2c2()
{
  if(i2c_flag[2] == 0)
  {
    i2c_flag[2] = 1;
  }
}

void isr_i2c3()
{
  if(i2c_flag[3] == 0)
  {
    i2c_flag[3] = 1;
  }
}

void isr_i2c4()
{
  if(i2c_flag[4] == 0)
  {
    i2c_flag[4] = 1;
  }
}

