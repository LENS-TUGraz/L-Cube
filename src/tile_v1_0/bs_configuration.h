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
#ifndef BS_CONFIGURATION_
#define BS_CONFIGURATION_


// method that handles normal configuration via serial interface
void perform_config_serial();

// method that handles quick configuration via serial interface
// no averaging of received angles as it is quick config -> if a more decent config is needed, use the normal one
void perform_quick_config_serial();

// method that handles normal configuration via i2c interface
void perform_config_i2c();

// method that handles quick configuration via i2c interface
// no averaging of received angles as it is quick config -> if a more decent config is needed, use the normal one
void perform_quick_config_i2c();

// starts recording data for normal configuration
void start_normal_config_record();

// turn off all front lights
void turn_lights_off();

// reset dirty bits of beams to ensure fresh values
void reset_dirty_bits();

#endif
