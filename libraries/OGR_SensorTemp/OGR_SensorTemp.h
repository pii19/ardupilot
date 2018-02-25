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
#define OGR_SENSORTEMP_MAX_INSTANCES 5
#define OGR_SENSORTEMP_PREARM_ALT_MAX_TEMP           150.0
#define OGR_SENSORTEMP_PREARM_ALT_MIN_TEMP           -55.0

class OGR_SensorTemp_Backend;

class OGR_SensorTemp
{
    friend class OGR_SensorTemp_Backend;

public:
    OGR_SensorTemp(void);

    /* Do not allow copies */
    OGR_SensorTemp(const OGR_SensorTemp &other) = delete;
    OGR_SensorTemp &operator=(const OGR_SensorTemp&) = delete;

    // sensor driver types
    enum OGR_SensorTemp_Type {
        OGR_SensorTemp_TYPE_NONE    = 0,
        OGR_SensorTemp_TYPE_ADT7410 = 1
    };

    enum OGR_SensorTemp_Status {
        OGR_SensorTemp_NotConnected = 0,
        OGR_SensorTemp_NoData,
        OGR_SensorTemp_OutOfRangeLow,
        OGR_SensorTemp_OutOfRangeHigh,
        OGR_SensorTemp_Good
    };

    // The OGR_SensorTemp_State structure is filled in by the backend driver
    struct OGR_SensorTemp_State {
        uint8_t                instance;    // the instance number of this OGR_SensorTemp
        float                  temperature; // temperature: in celsius
                                            // if applicable, otherwise 0
        enum OGR_SensorTemp_Status status;     // sensor status
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

    OGR_SensorTemp_Backend *get_backend(uint8_t id) const;

    /*
      returns true if pre-arm checks have passed for all temperature sensors
      these checks involve the user lifting or rotating the vehicle so that sensor readings between
      the min and 2m can be captured
     */
    bool pre_arm_check() const;


private:
    OGR_SensorTemp_State state[OGR_SENSORTEMP_MAX_INSTANCES];
    OGR_SensorTemp_Backend *drivers[OGR_SENSORTEMP_MAX_INSTANCES];
    uint8_t num_instances:1;

    void detect_instance(uint8_t instance);
    void update_instance(uint8_t instance);  

    bool _add_backend(OGR_SensorTemp_Backend *driver);
};
