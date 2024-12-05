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

// includeguard
#ifndef BS_CALIBRATION_
#define BS_CALIBRATION_

#include <stdint.h>

// we treat 16 bit floats as 16 bit number and do not perform calculations with them
typedef uint16_t fp_16;

// From https://github.com/nairol/LighthouseRedox/blob/master/docs/Base%20Station.md#base-station-info-block
// ootx calibration data structure
struct OotxCalibrationData_s {
  uint16_t protocolVersion:6;
  uint16_t firmwareVersion:10;
  uint32_t id;
  fp_16 phase0;
  fp_16 phase1;
  fp_16 tilt0;
  fp_16 tilt1;
  uint8_t unlockCount;
  uint8_t hwVersion;
  fp_16 curve0;
  fp_16 curve1;
  int8_t accelX;
  int8_t accelY;
  int8_t accelZ;
  fp_16 gibphase0;
  fp_16 gibphase1;
  fp_16 gibmag0;
  fp_16 gibmag1;
  uint8_t mode;
  uint8_t faults;

  // Only used in LH 2 that uses a longer data block
  fp_16 ogeephase0;
  fp_16 ogeephase1;
  fp_16 ogeemag0;
  fp_16 ogeemag1;
} __attribute__((packed));


// converts 16 bit float to 32 bit float (single)
float fp16_to_float(uint16_t f_16);

// the bitstream that is transferred from the lasers contains all bits twice (101 -> 110011), as the bits are emitted by both of ther lasers, one bit per round of the spinning drum
// -> thats why we have to wait for 34 zero-bits which correspond to the 17 zero-bit preamble
void record_calib(uint8_t bit);

// compare the two received streams -> if they match -> reduce to a clean bitstream (duplicated bits are removed) 
// -> save to calibration struct -> print it for header file
void compare_save_calibration();

// get calibration out of the received slow data bits
// tile has to be placed in the middle of the viewing range from the station, approximately 1-1.5m away
// function filters all bits of the 4 receiving photodiodes to receive one constant bitstream
void perform_calibration(uint8_t calib_bit, uint8_t station_index, uint8_t photodiode_index, uint32_t lfsr_index);

// apply calibration
// iterative approach (max iteration of 5)
// correct the received angles (horizontal angles from the two lasers of the base station, not azimuth and elevation!)
void correct_angles(float* raw_angle_0, float* raw_angle_1, float* corrected_angle_0, float* corrected_angle_1, uint8_t station_index);

// apply calibration model
float calibration_model(const float x, const float y, const float z, const float t, uint8_t station_index, uint8_t beam_index);

// get distortion for the current iteration
void get_distortion(uint8_t station_index, float* corrected_angle_0, float* corrected_angle_1, float* current_angle_0, float* current_angle_1);


#endif