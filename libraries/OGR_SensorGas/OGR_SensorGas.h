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
#define OGR_SENSORGAS_MAX_INSTANCES           1
#define OGR_SENSORGAS_PREARM_ALT_MAX_TEMP     100.0
#define OGR_SENSORGAS_PREARM_ALT_MIN_TEMP     -30.0
#define OGR_SENSORGAS_USE_CH                  4

class OGR_SensorGas_Backend;

class OGR_SensorGas
{
    friend class OGR_SensorGas_Backend;

public:
    OGR_SensorGas(void);

    /* Do not allow copies */
    OGR_SensorGas(const OGR_SensorGas &other) = delete;
    OGR_SensorGas &operator=(const OGR_SensorGas&) = delete;

    // sensor driver types
    enum OGR_SensorGas_Type {
        OGR_SensorGas_TYPE_NONE    = 0,
        OGR_SensorGas_TYPE_ADS1015 = 1
    };

    enum OGR_SensorGas_Status {
        OGR_SensorGas_NotConnected = 0,
        OGR_SensorGas_NoData,
        OGR_SensorGas_OutOfRangeLow,
        OGR_SensorGas_OutOfRangeHigh,
        OGR_SensorGas_Good
    };

    // The OGR_SensorGas_State structure is filled in by the backend driver
    struct OGR_SensorGas_State {
        uint8_t                instance;    // the instance number of this OGR_SensorGas
        float                  temperature[OGR_SENSORGAS_USE_CH]; // temperature: in celsius
        float                  voltage[OGR_SENSORGAS_USE_CH];  // voltage

        enum OGR_SensorGas_Status status;     // sensor status
        uint8_t                valid_count;   // number of consecutive valid readings (maxes out at 10)
        bool                   pre_arm_check;   // true if sensor has passed pre-arm checks
        float                  pre_arm_temperature_min;    // min temperature captured during pre-arm checks
        float                  pre_arm_temperature_max;    // max temperature captured during pre-arm checks

        AP_Int8  type;
        AP_Int8  address;
        AP_Float min_temperature[OGR_SENSORGAS_USE_CH];
        AP_Float max_temperature[OGR_SENSORGAS_USE_CH];
        AP_Int8  ch[OGR_SENSORGAS_USE_CH]; //using ADC channel number
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

    OGR_SensorGas_Backend *get_backend(uint8_t id) const;

    /*
      returns true if pre-arm checks have passed for all temperature sensors
      these checks involve the user lifting or rotating the vehicle so that sensor readings between
      the min and 2m can be captured
     */
    bool pre_arm_check() const;


private:
    OGR_SensorGas_State state[OGR_SENSORGAS_MAX_INSTANCES];
    OGR_SensorGas_Backend *drivers[OGR_SENSORGAS_MAX_INSTANCES];
    uint8_t num_instances:1;

    void detect_instance(uint8_t instance);
    void update_instance(uint8_t instance);  

    bool _add_backend(OGR_SensorGas_Backend *driver);
};
