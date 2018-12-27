/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

// Maximum number of temperature sensor instances available on this platform
#define WJF_SENSORTEMPHUMI_MAX_INSTANCES 1
#define WJF_SENSORTEMPHUMI_PREARM_ALT_MAX_TEMP           125.0
#define WJF_SENSORTEMPHUMI_PREARM_ALT_MIN_TEMP           -40.0

class WJF_SensorTempHumi_Backend;

class WJF_SensorTempHumi
{
    friend class WJF_SensorTempHumi_Backend;

public:
    WJF_SensorTempHumi(void);

    /* Do not allow copies */
    WJF_SensorTempHumi(const WJF_SensorTempHumi &other) = delete;
    WJF_SensorTempHumi &operator=(const WJF_SensorTempHumi&) = delete;

    // sensor driver types
    enum WJF_SensorTempHumi_Type {
        WJF_SensorTempHumi_TYPE_NONE    = 0,
        WJF_SensorTempHumi_TYPE_SHT31D = 1,
    };

    enum WJF_SensorTempHumi_Status {
        WJF_SensorTempHumi_NotConnected = 0,
        WJF_SensorTempHumi_NoData,
        WJF_SensorTempHumi_OutOfRangeLow,
        WJF_SensorTempHumi_OutOfRangeHigh,
        WJF_SensorTempHumi_Good
    };

    // The WJF_SensorTempHumi_State structure is filled in by the backend driver
    struct WJF_SensorTempHumi_State {
        uint8_t                instance;    // the instance number of this WJF_SensorTempHumi
        float                  temperature; // temperature: in celsius
        float                  humidity; // humidity
                                            // if applicable, otherwise 0
        enum WJF_SensorTempHumi_Status status;     // sensor status
        uint8_t                valid_count;   // number of consecutive valid readings (maxes out at 10)
        bool                   pre_arm_check;   // true if sensor has passed pre-arm checks
        float                  pre_arm_temperature_min;    // min temperature captured during pre-arm checks
        float                  pre_arm_temperature_max;    // max temperature captured during pre-arm checks

        AP_Int8  type;
        AP_Float min_temperature;
        AP_Float max_temperature;
        AP_Int8  address;
    };

    // parameters for each instance
    static const struct AP_Param::GroupInfo var_info[];
    
    // Return the number of temperature sensor instances
    uint8_t num_sensors(void) const {
        return num_instances;
    }

    // detect and initialise any available temperature sensors
    void init(void);

    // update state of all temperature sensors. Should be called at around
    // 10Hz from main loop
    void update(void);

    WJF_SensorTempHumi_Backend *get_backend(uint8_t id) const;

    /*
      returns true if pre-arm checks have passed for all temperature sensors
      these checks involve the user lifting or rotating the vehicle so that sensor readings between
      the min and 2m can be captured
     */
    bool pre_arm_check() const;


private:
    WJF_SensorTempHumi_State state[WJF_SENSORTEMPHUMI_MAX_INSTANCES];
    WJF_SensorTempHumi_Backend *drivers[WJF_SENSORTEMPHUMI_MAX_INSTANCES];
    uint8_t num_instances:1;

    void detect_instance(uint8_t instance);
    void update_instance(uint8_t instance);  

    bool _add_backend(WJF_SensorTempHumi_Backend *driver);
};
