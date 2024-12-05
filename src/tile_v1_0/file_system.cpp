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

#include "LittleFS.h"
#include "config.h"
#include "mmath.h"

// station positions and heading
extern vec3d station_positions[MAX_BASE_STATIONS];
extern mat3x3 station_orientations[MAX_BASE_STATIONS];

// file pointer
File f;
// current line that is read
String line;
// string that contains the station number
String station_number_string;
// substrings that contain configuration values
String sub_strings[MAX_SUBSTRINGS_CONFIG];
// counter for substrings
int string_count = 0;
// current station number
int station_number = 0;


// load the config file or init the file if it is not present
void init_load_config_file()
{
  LittleFS.begin();
  if(LittleFS.exists("/bs_config.csv"))
  {
    // load config data
    // read from file
    f = LittleFS.open("/bs_config.csv", "r");
    if (f)
    {
      int line_count = 0;

      // omit first line (is the csv header)
      if(f.available())
      {
        // already strips '\n'
        line = f.readStringUntil('\n');
      }
      while (f.available())
      {
        // already strips \n
        line = f.readStringUntil('\n');

        #if (DBG_CONFIG_FILE == 1)
          // print file data
          line_count++;
          Serial.print("[DBG_CONFIG_FILE] Line ");
          Serial.print(line_count);
          Serial.print(": ");
          Serial.println(line);
        #endif

        // parse string
        // get station number
        if(line.length() > 0)
        {
          int index = line.indexOf(',');
          if (index == -1)
          {
            // invalid line
            continue;
          }
          else
          {
            // get station number out of the current configuration line
            station_number_string = line.substring(0, index);
            station_number = station_number_string.toInt();
            line = line.substring(index + 1);
          }
        }

        // split float values into substrings
        string_count = 0;
        while((line.length() > 0) && (string_count < MAX_SUBSTRINGS_CONFIG))
        {
          int index = line.indexOf(',');
          // no separator found
          if (index == -1)
          {
            sub_strings[string_count++] = line;
            break;
          }
          else
          {
            sub_strings[string_count++] = line.substring(0, index);
            line = line.substring(index + 1);
          }
        }
        // convert to floats if all data there
        if(string_count == MAX_SUBSTRINGS_CONFIG)
        {
          for(int i = 0; i < MAX_SUBSTRINGS_CONFIG; ++i)
          {
            if(i < 3)
            {
              station_positions[station_number][i] = sub_strings[i].toFloat();
            }
            else if(i < 6)
            {
              station_orientations[station_number][0][i-3] = sub_strings[i].toFloat();
            }
            else if(i < 9)
            {
              station_orientations[station_number][1][i-6] = sub_strings[i].toFloat();
            }
            else
            {
              station_orientations[station_number][2][i-9] = sub_strings[i].toFloat();
            }
          }
        }
      }
      f.close();
    }
    else
    {
      #if (DBG_CONFIG_FILE == 1)
        Serial.println("[DBG_CONFIG_FILE] File open for reading failed");
      #endif
    }
  }
  else
  {
    #if (DBG_CONFIG_FILE == 1)
      Serial.println("[DBG_CONFIG_FILE] Init empty file");
    #endif
    f = LittleFS.open("/bs_config.csv", "w");
    if (f)
    {
      // write file with only csv header
      f.write("station,position_x,position_y,position_z,rotation_00,rotation_01,rotation_02,rotation_10,rotation_11,rotation_12,rotation_20,rotation_21,rotation_22\n");
      f.close();
    }
    else
    {
      #if (DBG_CONFIG_FILE == 1)
        Serial.println("[DBG_CONFIG_FILE] File open for init file failed");
      #endif
    }
  }
  // unmount fs
  LittleFS.end();
}


// store the current configuration in a file
void store_config_file()
{
  //mount fs
  LittleFS.begin();

  f = LittleFS.open("/bs_config.csv", "w");
  if (f)
  {
    String file_to_write = String("");
    f.write("station,position_x,position_y,position_z,rotation_00,rotation_01,rotation_02,rotation_10,rotation_11,rotation_12,rotation_20,rotation_21,rotation_22\n");
    for(int file_station_index = 0; file_station_index < 16; ++file_station_index)
    {
      // create content to write to file
      // 30 bytes should be enough for 20 comma positions
      char buffer[40];

      file_to_write += String(file_station_index);
      file_to_write += String(",");
      // position
      sprintf(buffer, "%0.20f,", station_positions[file_station_index][0]);
      file_to_write += String(buffer);
      sprintf(buffer, "%0.20f,", station_positions[file_station_index][1]);
      file_to_write += String(buffer);
      sprintf(buffer, "%0.20f,", station_positions[file_station_index][2]);
      file_to_write += String(buffer);
      // heading
      sprintf(buffer, "%0.20f,", station_orientations[file_station_index][0][0]);
      file_to_write += String(buffer);
      sprintf(buffer, "%0.20f,", station_orientations[file_station_index][0][1]);
      file_to_write += String(buffer);
      sprintf(buffer, "%0.20f,", station_orientations[file_station_index][0][2]);
      file_to_write += String(buffer);
      sprintf(buffer, "%0.20f,", station_orientations[file_station_index][1][0]);
      file_to_write += String(buffer);
      sprintf(buffer, "%0.20f,", station_orientations[file_station_index][1][1]);
      file_to_write += String(buffer);
      sprintf(buffer, "%0.20f,", station_orientations[file_station_index][1][2]);
      file_to_write += String(buffer);
      sprintf(buffer, "%0.20f,", station_orientations[file_station_index][2][0]);
      file_to_write += String(buffer);
      sprintf(buffer, "%0.20f,", station_orientations[file_station_index][2][1]);
      file_to_write += String(buffer);
      sprintf(buffer, "%0.20f\n", station_orientations[file_station_index][2][2]);
      file_to_write += String(buffer);
      
    }
    f.write(file_to_write.c_str());
    f.close();
  }
  else
  {
    #if (DBG_CONFIG_FILE == 1)
      Serial.println("[DBG_CONFIG_FILE] file open for writing failed");
    #endif
  }

  // unmount fs
  LittleFS.end();
}
