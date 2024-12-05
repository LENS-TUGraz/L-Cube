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

// data file for bs_calibration.cpp (include there!)

typedef struct CalibrationData_s{
  // used
  float phase0;
  float phase1;
  float tilt0;
  float tilt1;
  float gibphase0;
  float gibphase1;
  float gibmag0;
  float gibmag1;
  // not used
  float curve0;
  float curve1;
  float ogeephase0;
  float ogeephase1;
  float ogeemag0;
  float ogeemag1;
  uint8_t unlockCount;
  uint8_t hwVersion;
  int8_t accelX;
  int8_t accelY;
  int8_t accelZ;
  uint8_t mode;
  uint8_t faults;
  uint32_t id;
} CalibrationData;

/* in here, the lines can be replaced with the ones from the calibration */
CalibrationData calibration_data[MAX_BASE_STATIONS] = 
{
  // CHANNEL 1 (id0)
  {0.0000000000, -0.0048446655, -0.0487060547, 0.0459594727, 2.6601562500, 1.5869140625, -0.0010652542, -0.0005722046, 0.1735839844, 0.3967285156, 1.1308593750, 1.6376953125, -0.2714843750, -0.2265625000, 1, 14, 1, 127, 91, 130, 0, 3829856027},
  // CHANNEL 2 (id1)
  {0.0000000000, -0.0047531128, -0.0502014160, 0.0428161621, 0.4416503906, 1.8173828125, 0.0033779144, 0.0026893616, 0.2082519531, 0.2298583984, 0.9404296875, 1.6357421875, -0.4470214844, -0.4428710938, 5, 14, 0, 49, 127, 129, 0, 2253564684},
  // CHANNEL 3 (id2)
  {0.0000000000, -0.0061340332, -0.0445556641, 0.0439758301, 1.2197265625, 1.8193359375, -0.0108261108, -0.0121994019, 0.1912841797, 0.2319335938, 1.1962890625, 1.7363281250, 0.3215332031, 0.4086914063, 1, 14, 4, 8, 127, 130, 0, 2127893353},
  // CHANNEL 4 (id3
  {0.0000000000, -0.0001009107, -0.0422668457, 0.0491638184, 0.8027343750, 1.8886718750, 0.0047607422, 0.0034027100, -0.2225341797, 0.4572753906, 1.1152343750, 1.5302734375, -0.4726562500, -0.4362792969, 1, 14, -4, 1, 127, 131, 0, 2982525517},
  // CHANNEL 5 (id4)
  {0.0000000000, -0.0057334900, -0.0403137207, 0.0464172363, 0.4658203125, 1.4287109375, 0.0064430237, 0.0052795410, 0.1145019531, 0.3657226563, 0.9091796875, 1.4501953125, -0.5283203125, -0.5703125000, 4, 14, -1, 127, 63, 129, 0, 3036314002},
  // CHANNEL 6 (id5)
  {0.0000000000, -0.0158233643, -0.0475463867, 0.0440368652, 0.7773437500, 2.4453125000, 0.0045433044, 0.0034103394, -0.0132217407, -0.2705078125, 1.2763671875, 2.1386718750, -0.4553222656, -0.4169921875, 1, 14, 0, 127, 61, 128, 0, 1123484766},
  // if channel is switched to 1 and calibration is recorded again -> only AccelX/Y/Z, mode, unlock count change values -> we do not use them, so changing these lines when channels are changed is possible
  {},
  {}
};

