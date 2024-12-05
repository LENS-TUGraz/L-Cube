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

#include "isr.h"
#include <Arduino.h>
#include "config.h"

extern uint8_t e_flag[MAX_PHOTODIODES];

void isr_e0()
{
  if(e_flag[0] == 0)
  {
    e_flag[0] = 1;
  }
}

void isr_e1()
{
  if(e_flag[1] == 0)
  {
    e_flag[1] = 1;
  }
}

void isr_e2()
{
  if(e_flag[2] == 0)
  {
    e_flag[2] = 1;
  }
}

void isr_e3()
{
  if(e_flag[3] == 0)
  {
    e_flag[3] = 1;
  }
}

