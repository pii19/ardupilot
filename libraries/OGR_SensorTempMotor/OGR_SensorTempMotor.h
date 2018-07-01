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
#define OGR_SENSORTEMPMOTOR_MAX_INSTANCES           1
#define OGR_SENSORTEMPMOTOR_PREARM_ALT_MAX_TEMP     100.0
#define OGR_SENSORTEMPMOTOR_PREARM_ALT_MIN_TEMP     -30.0
#define OGR_SENSORTEMPMOTOR_USE_CH                  4

class OGR_SensorTempMotor_Backend;

class OGR_SensorTempMotor
{
    friend class OGR_SensorTempMotor_Backend;

public:
    OGR_SensorTempMotor(void);

    /* Do not allow copies */
    OGR_SensorTempMotor(const OGR_SensorTempMotor &other) = delete;
    OGR_SensorTempMotor &operator=(const OGR_SensorTempMotor&) = delete;

    // sensor driver types
    enum OGR_SensorTempMotor_Type {
        OGR_SensorTempMotor_TYPE_NONE    = 0,
        OGR_SensorTempMotor_TYPE_ADS1015_S8120C = 1
    };

    enum OGR_SensorTempMotor_Status {
        OGR_SensorTempMotor_NotConnected = 0,
        OGR_SensorTempMotor_NoData,
        OGR_SensorTempMotor_OutOfRangeLow,
        OGR_SensorTempMotor_OutOfRangeHigh,
        OGR_SensorTempMotor_Good
    };

    // The OGR_SensorTempMotor_State structure is filled in by the backend driver
    struct OGR_SensorTempMotor_State {
        uint8_t                instance;    // the instance number of this OGR_SensorTempMotor
        float                  temperature[OGR_SENSORTEMPMOTOR_USE_CH]; // temperature: in celsius
        float                  voltage[OGR_SENSORTEMPMOTOR_USE_CH];  // voltage

        enum OGR_SensorTempMotor_Status status;     // sensor status
        uint8_t                valid_count;   // number of consecutive valid readings (maxes out at 10)
        bool                   pre_arm_check;   // true if sensor has passed pre-arm checks
        float                  pre_arm_temperature_min;    // min temperature captured during pre-arm checks
        float                  pre_arm_temperature_max;    // max temperature captured during pre-arm checks

        AP_Int8  type;
        AP_Int8  address;
        AP_Float min_temperature[OGR_SENSORTEMPMOTOR_USE_CH];
        AP_Float max_temperature[OGR_SENSORTEMPMOTOR_USE_CH];
        AP_Int8  ch[OGR_SENSORTEMPMOTOR_USE_CH]; //using ADC channel number
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

    OGR_SensorTempMotor_Backend *get_backend(uint8_t id) const;

    /*
      returns true if pre-arm checks have passed for all temperature sensors
      these checks involve the user lifting or rotating the vehicle so that sensor readings between
      the min and 2m can be captured
     */
    bool pre_arm_check() const;


private:
    OGR_SensorTempMotor_State state[OGR_SENSORTEMPMOTOR_MAX_INSTANCES];
    OGR_SensorTempMotor_Backend *drivers[OGR_SENSORTEMPMOTOR_MAX_INSTANCES];
    uint8_t num_instances:1;

    void detect_instance(uint8_t instance);
    void update_instance(uint8_t instance);  

    bool _add_backend(OGR_SensorTempMotor_Backend *driver);
};
