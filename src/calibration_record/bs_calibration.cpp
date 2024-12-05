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

#include "bs_calibration.h"
#include "config.h"
#include "bs_calibration_data.h"
#include <Arduino.h>

union {
    uint32_t  u;
    float     f;
} fp16_converter;


// floating point 16 to float (single) converter
float fp16_to_float(uint16_t f_16)
{
  if(f_16 == 0)
  {
    return 0.0;
  }
  uint32_t tmp = f_16;
  //                   sign                      exponent bits + 0111 0000          mantissa
  fp16_converter.u = ((tmp & 0x8000) << 16) | (((tmp & 0x7c00) + 0x1C000) << 13) | ((tmp & 0x03FF) << 13);
  return fp16_converter.f;
}


// alternates between 0 for first laser (first beam) and 1 for second laser (second beam)
uint8_t curent_laser_index = 0;
// timestamp to wait some ms until next possible bit can occur
unsigned long calib_timestamp = 0;
// amount of how much bits are logged per second (should be 100, otherwise beam reception is bad)
extern uint8_t call_counter;



//                                            (ootx extends payload to even number)
// total length of stream: 2 bytes length + 43(+1) bytes payload + 4 bytes crc = 50 bytes
// stuff bits (every 17th bit, so after every 16th bit (2 bytes) there is a stuff bit): 50 byte/2 = 25 stuff bits + 1 stuff bit at beginning that we log
// in total: 50 bytes + 26 bits = 53,25 bytes
// -> we receive every bit twice (is present in bot lasers per round) -> 53,25 * 2 = 106,5 bytes -> 852 bits

// 886 bits for calibration, including 34 0-bits preamble which we do not log
#define BITS_PER_CALIBRATION 852
// 852 / 8 = 106.5 ->> !! in total should be 106,25 bytes (+0,25 bytes for first duplicated 1 of preamble) received if all bits are doubled, as they are sent
#define BYTES_PER_CALIBRATION 107
// length of the pure payload in bytes without stuffing bits
#define CALIBRATION_LENGTH 43

// decoder struct
typedef struct s_CalibrationDecoder {
  // 2 recorded bitstreams to compare calibration
  uint8_t bitstreams[2][BYTES_PER_CALIBRATION] = {0};
  // currently recorded bitstream
  uint8_t current_bitstream = 0;
  
  uint8_t zero_counter = 0;
  uint8_t save_bits = 0;
  uint32_t received_bits = 0;
  uint8_t calibration_done = 0;

  uint8_t data_stream[BYTES_PER_CALIBRATION/2] = {0};

  union {
    uint8_t data[CALIBRATION_LENGTH];
    struct OotxCalibrationData_s frame;
  };
} CalibrationDecoder;

// decoder struct
CalibrationDecoder decoder;

// get calibration out of the received slow data bits
// tile has to be placed in the middle of the viewing range from the station, approximately 1-1.5m away
// function filters all bits of the 4 receiving photodiodes to receive one constant bitstream
void perform_calibration(uint8_t calib_bit, uint8_t station_index, uint8_t photodiode_index, uint32_t lfsr_index)
{
  if(curent_laser_index == 0)
  {
    // need to receive index from next laser & specific time has to pass to change laser index
    if(lfsr_index > 57500) // 57500 is approximately the middle between the laser indexes
    {
      // x ms have to pass so it is physically possible that next laser is received
      if((millis() - calib_timestamp) > 1) 
      {
        // we got new bit, evaluate it!
        record_calib(calib_bit);
        // contains amount of calls of record function in one second (should be 100), otherwise reception of beams is bad
        call_counter ++;

        curent_laser_index = 1;
        calib_timestamp = millis();
      }
    }
  }
  else
  {
    // need to receive index from next laser & specific time has to pass to change laser index
    if(lfsr_index < 57500) // 57500 is approximately the middle between the laser indexes
    {
      // x ms have to pass so it is physically possible that next laser is received
      if((millis() - calib_timestamp) > 1) 
      {
        // we got new bit, evaluate it!
        record_calib(calib_bit);
        // contains amount of calls of record function in one second (should be 100), otherwise reception of beams is bad
        call_counter ++;

        curent_laser_index = 0;
        calib_timestamp = millis();
      }
    }
  }
}



// the bitstream that is transferred from the lasers contains all bits twice (101 -> 110011), as the bits are emitted by both of ther lasers, one bit per round of the spinning drum
// -> thats why we have to wait for 34 zero-bits which correspond to the 17 zero-bit preamble
void record_calib(uint8_t bit)
{
  // 34 zero bits as preamble
  if(decoder.zero_counter > 33)
  {
    // start logging of stream
    if(bit == 1)
    {
      // stream has 110,75 bytes -> 886 bits in one full cycle!
      decoder.received_bits = 0;
      decoder.save_bits = 1;
    }
  }

  // preamble counter
  if(bit == 0)
  {
    decoder.zero_counter ++;
  }
  else
  {
    decoder.zero_counter = 0;
  }

  // logging bits
  if(decoder.save_bits == 1)
  {
    // create bitstream
    decoder.bitstreams[decoder.current_bitstream][decoder.received_bits/8] = (decoder.bitstreams[decoder.current_bitstream][decoder.received_bits/8] << 1) | bit;
    decoder.received_bits ++;

    // stream including stuff bits and first bit (all duplicated)
    if(decoder.received_bits == BITS_PER_CALIBRATION)
    {
      #if (DBG_CALIB == 1)
        Serial.print("[DBG_CALIB] Received stream ");
        Serial.println(decoder.current_bitstream);
      #endif

      if(decoder.current_bitstream == 1)
      {
        // compare both consecutive received bitstreams and save to struct if they match
        compare_save_calibration();
        decoder.current_bitstream = 0;
      }
      else
      {
        decoder.current_bitstream = 1;
      }
      
      // stop recording bits, as all are received
      decoder.save_bits = 0;
    }
  }
}



// compare the two received streams -> if the match -> reduce to a clean bitstream (duplicated bits are removed) 
// -> save to calibration struct -> print it for header file
void compare_save_calibration()
{
  // for deduplication of bits
  uint8_t data_bit = 0;
  uint32_t data_position = 0;
  // if stream has stuffing error (every 17th bit in clean stream is 1)
  uint8_t stuff_error = 0;
  // length of payload from received stream
  uint16_t payload_length = 0;
  
  #if (DBG_CALIB == 1)
    Serial.println("[DBG_CALIB] Comparing two bitstreams");
  #endif

  // compare bitstreams
  for(int i = 0; i < BYTES_PER_CALIBRATION; ++i)
  {
    if(decoder.bitstreams[decoder.current_bitstream][i] != decoder.bitstreams[decoder.current_bitstream][i])
    {
      #if (DBG_CALIB == 1)
        Serial.println("[DBG_CALIB] No match at index");
        Serial.println(i);
      #endif
      // does not match, keep recording new streams
      return;
    }
  }

  // calibration does match -> save calibration and check stuffing
  #if (DBG_CALIB == 1)
    Serial.println("[DBG_CALIB] Bitstreams match");
  #endif

  // dedupication of bits
  for(int i = 0; i < BITS_PER_CALIBRATION; i += 2)
  {
    data_bit = (decoder.bitstreams[0][i/8] & (0x80 >> (i % 8)));
    
    // create bit for shifting into new stream
    if(data_bit != 0)
    {
      // move bit to first position (lsb)
      data_bit = 1;
    }

    // first bit is stuff bit, then every 17th bit is stuff bit
    // check stuff bits
    if((i % 17) == 0)
    {
      if(data_bit != 1)
      {
        #if (DBG_CALIB == 1)
          // for the first reception, a stuff error occurs at position 850
          // this is because the last byte is not fully received and shifted, as we only receive 106.5 bytes (0x03 instead of 0x30)
          // however in the the next receiving cycle this problem gets mittigated (it just takes a bit longer to receive calibration)
          Serial.print("[DBG_CALIB] Stuff error at position ");
          Serial.println(i);
        #endif
        
        stuff_error = 1;
      }
      continue;
    }

    // create clean bitstream
    decoder.data_stream[data_position/8] = (decoder.data_stream[data_position/8] << 1) | data_bit;
    data_position++;
  }
  
  // print clean stream
  #if (DBG_CALIB == 1)
    Serial.println("[DBG_CALIB] Clean bitstream");
    for(int i = 0; i < BYTES_PER_CALIBRATION/2; ++i)
    {
      Serial.print(decoder.data_stream[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  #endif

  // rp2040 is little endian, and stream is received in little endian, so no conversion needed
  payload_length = *((uint16_t*)&decoder.data_stream[0]);
  
  // calibration for lighthouse v2 is always 43 bytes
  if(payload_length != 43)
  {
    #if (DBG_CALIB == 1)
      Serial.print("[DBG_CALIB] Payload length of received calib stream is wrong: ");
      Serial.println(payload_length);
    #endif
    return;
  }

  // if this point is reached, the received bitstream is correct -> copy data stream to struct
  // the data stream would have crc32 in it as well, but we do not use the checksum, as we compare two consecutive received stream
  // first 2 bytes are length, then the payload follows, then the crc32 would be present
  // we only need payload here
  memcpy(decoder.data, &decoder.data_stream[2], CALIBRATION_LENGTH);

  // float conversion
  float phase_0 = fp16_to_float(decoder.frame.phase0);
  float phase_1 = fp16_to_float(decoder.frame.phase1);
  float tilt_0 = fp16_to_float(decoder.frame.tilt0);
  float tilt_1 = fp16_to_float(decoder.frame.tilt1);
  float gibphase_0 = fp16_to_float(decoder.frame.gibphase0);
  float gibphase_1 = fp16_to_float(decoder.frame.gibphase1);
  float gibmag_0 = fp16_to_float(decoder.frame.gibmag0);
  float gibmag_1 = fp16_to_float(decoder.frame.gibmag1);
  float curve_0 = fp16_to_float(decoder.frame.curve0);
  float curve_1 = fp16_to_float(decoder.frame.curve1);
  float ogeephase_0 = fp16_to_float(decoder.frame.ogeephase0);
  float ogeephase_1 = fp16_to_float(decoder.frame.ogeephase1);
  float ogeemag_0 = fp16_to_float(decoder.frame.ogeemag0);
  float ogeemag_1 = fp16_to_float(decoder.frame.ogeemag1);

  // print the data
#if (DBG_CALIB == 1)
  // READABLE DATA FORMAT
  String calib_str;
  Serial.println("[DBG_CALIB] === CALIB DATA ===");
  Serial.println("Protocol Version:");
  Serial.println(decoder.frame.protocolVersion);
  Serial.println("Firmware Version:");
  Serial.println(decoder.frame.firmwareVersion);
  Serial.println("ID:");
  Serial.println(decoder.frame.id);
  Serial.println("Phase 0:");
  calib_str = String(phase_0, 10);
  Serial.println(calib_str);
  Serial.println("Phase 1:");
  calib_str = String(phase_1, 10);
  Serial.println(calib_str);
  Serial.println("Tilt 0:");
  calib_str = String(tilt_0, 10);
  Serial.println(calib_str);
  Serial.println("Tilt 1:");
  calib_str = String(tilt_1, 10);
  Serial.println(calib_str);
  Serial.println("Unlock Count:");
  Serial.println(decoder.frame.unlockCount);
  Serial.println("HW Version:");
  Serial.println(decoder.frame.hwVersion);
  Serial.println("Curve 0:");
  calib_str = String(curve_0, 10);
  Serial.println(calib_str);
  Serial.println("Curve 1:");
  calib_str = String(curve_1, 10);
  Serial.println(calib_str);
  Serial.println("Accel X:");
  Serial.println(decoder.frame.accelX);
  Serial.println("Accel Y:");
  Serial.println(decoder.frame.accelY);
  Serial.println("Accel Z:");
  Serial.println(decoder.frame.accelZ);
  Serial.println("Gibphase 0:");
  calib_str = String(gibphase_0, 10);
  Serial.println(calib_str);
  Serial.println("Gibphase 1:");
  calib_str = String(gibphase_1, 10);
  Serial.println(calib_str);
  Serial.println("Gibmag 0:");
  calib_str = String(gibmag_0, 10);
  Serial.println(calib_str);
  Serial.println("Gibmag 1:");
  calib_str = String(gibmag_1, 10);
  Serial.println(calib_str);
  Serial.println("Mode:");
  Serial.println(decoder.frame.mode);
  Serial.println("Faults:");
  Serial.println(decoder.frame.faults);
  Serial.println("Ogeephase 0:");
  calib_str = String(ogeephase_0, 10);
  Serial.println(calib_str);
  Serial.println("Ogeephase 1:");
  calib_str = String(ogeephase_1, 10);
  Serial.println(calib_str);
  Serial.println("Ogeemag 0:");
  calib_str = String(ogeemag_0, 10);
  Serial.println(calib_str);
  Serial.println("Ogeemag 1:");
  calib_str = String(ogeemag_1, 10);
  Serial.println(calib_str);
#endif

  // header format, to use in code then
  String float_str;
  Serial.println("--- Calibration header string ---");
  // data for header file
  Serial.print("{");
  float_str = String(phase_0, 10);
  Serial.print(float_str);
  Serial.print(", ");
  float_str = String(phase_1, 10);
  Serial.print(float_str);
  Serial.print(", ");
  float_str = String(tilt_0, 10);
  Serial.print(float_str);
  Serial.print(", ");
  float_str = String(tilt_1, 10);
  Serial.print(float_str);
  Serial.print(", ");
  float_str = String(gibphase_0, 10);
  Serial.print(float_str);
  Serial.print(", ");
  float_str = String(gibphase_1, 10);
  Serial.print(float_str);
  Serial.print(", ");
  float_str = String(gibmag_0, 10);
  Serial.print(float_str);
  Serial.print(", ");
  float_str = String(gibmag_1, 10);
  Serial.print(float_str);
  Serial.print(", ");
  float_str = String(curve_0, 10);
  Serial.print(float_str);
  Serial.print(", ");
  float_str = String(curve_1, 10);
  Serial.print(float_str);
  Serial.print(", ");
  float_str = String(ogeephase_0, 10);
  Serial.print(float_str);
  Serial.print(", ");
  float_str = String(ogeephase_1, 10);
  Serial.print(float_str);
  Serial.print(", ");
  float_str = String(ogeemag_0, 10);
  Serial.print(float_str);
  Serial.print(", ");
  float_str = String(ogeemag_1, 10);
  Serial.print(float_str);
  Serial.print(", ");
  Serial.print(decoder.frame.unlockCount);
  Serial.print(", ");
  Serial.print(decoder.frame.hwVersion);
  Serial.print(", ");
  Serial.print(decoder.frame.accelX);
  Serial.print(", ");
  Serial.print(decoder.frame.accelY);
  Serial.print(", ");
  Serial.print(decoder.frame.accelZ);
  Serial.print(", ");
  Serial.print(decoder.frame.mode);
  Serial.print(", ");
  Serial.print(decoder.frame.faults);
  Serial.print(", ");
  Serial.print(decoder.frame.id);
  Serial.println("},");
  
  Serial.println("--- Calibration header string finished ---");

  #if (DBG_CALIB == 1)
    Serial.println("[DBG_CALIB] Raw received stream");
    // print bitstream
    // the last two bits are 0 -> but were not received!! (only 110,75 bytes to receive in one cycle)
    // -> nibble of the last byte in other nibble (0x03 instead of 0x30) -> but this is stuff bit anyways, so we do not use it
    for(int i = 0; i < BYTES_PER_CALIBRATION; ++i)
    {
      Serial.print(decoder.bitstreams[decoder.current_bitstream][i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  #endif

  if(stuff_error == 0)
  {
    // mark calibration as decoded
    // not used
  }
}

// calibration usage:
// could be used for extension later!
//# calibration used in libsurvive: https://github.com/cntools/libsurvive/blob/master/src/survive_reproject_gen2.c#L114
//# calibration used in cf: https://www.bitcraze.io/about/events/documents/tutorial_demo_presentation.pdf



// ============================================================================================================================================================
// ===== CALIBRATION USAGE TO CORRECT ANGLES - FROM CRAZYFLIE =====
// ============================================================================================================================================================

//angle 0 is angle of first beam (with lower index), angle 1 is angle of second beam
void correct_angles(float* raw_angle_0, float* raw_angle_1, float* corrected_angle_0, float* corrected_angle_1, uint8_t station_index)
{
  const float max_delta = 0.0005;

  // Use distorted angle as a starting point
  *corrected_angle_0 = *raw_angle_0;
  *corrected_angle_1 = *raw_angle_1;

  for (int i = 0; i < 5; i++)
  {
    float current_distorted_angle_0;
    float current_distorted_angle_1;
    
    get_distortion(station_index, corrected_angle_0, corrected_angle_1, &current_distorted_angle_0, &current_distorted_angle_1);

    float delta_angle_0 = *raw_angle_0 - current_distorted_angle_0;
    float delta_angle_1 = *raw_angle_1 - current_distorted_angle_1;

    *corrected_angle_0 = *corrected_angle_0 + delta_angle_0;
    *corrected_angle_1 = *corrected_angle_1 + delta_angle_1;
    
    // early exit, if nearly converged
    if (fabs((double)delta_angle_0) < max_delta && fabs((double)delta_angle_1) < max_delta) 
    {
      break;
    }
  }
}

void get_distortion(uint8_t station_index, float* corrected_angle_0, float* corrected_angle_1, float* current_distorted_angle_0, float* current_distorted_angle_1)
{
  // use cos(x), sin(x)
  const float t30 = PI / 6.0f;
  // const float tan30 = tanf(t30);
  const float tan30 = 0.5773502691896258;  

  const float a0 = *corrected_angle_0;
  const float a1 = *corrected_angle_1;

  const float x = 1.0f;
  const float y = tanf((a1 + a0) / 2.0f);
  const float z = sinf(a1 - a0) / (tan30 * (cosf(a1) + cosf(a0)));

  // index 0 is first beam
  *current_distorted_angle_0 = calibration_model(x, y, z, -t30, station_index, 0);
  // index 1 in second beam
  *current_distorted_angle_1 = calibration_model(x, y, z, t30, station_index, 1);
}


float clip_1(float a)
{
  if (a < -1.0f) 
  {
    return -1.0f;
  }

  if (a > 1.0f) 
  {
    return 1.0f;
  }

  return a;
}

// beam index: 0 for first beam (from first laser), 1 for second beam (second laser on drum)
float calibration_model(const float x, const float y, const float z, const float t, uint8_t station_index, uint8_t beam_index)
{
  if(beam_index == 0)
  {
    const float ax = atan2f(y, x);
    // const float ay = atan2f(z, x);
    const float r = sqrt(x * x + y * y);

    const float base = ax + asinf(clip_1(z * tanf(t - calibration_data[station_index].tilt0) / r));
    const float compGib = -calibration_data[station_index].gibmag0 * cos(ax + calibration_data[station_index].gibphase0);
    
    // crazyflie also still tries to find an approach on the useage of curve and ogee parameters
    return base - (calibration_data[station_index].phase0 + compGib);
  }
  if(beam_index == 1)
  {
    const float ax = atan2f(y, x);
    // const float ay = atan2f(z, x);
    const float r = sqrt(x * x + y * y);

    const float base = ax + asinf(clip_1(z * tanf(t - calibration_data[station_index].tilt1) / r));
    const float compGib = -calibration_data[station_index].gibmag1 * cos(ax + calibration_data[station_index].gibphase1);
    
    // crazyflie also still tries to find an approach on the useage of curve and ogee parameters
    return base - (calibration_data[station_index].phase1 + compGib);
  }

  return 0.0;
}
