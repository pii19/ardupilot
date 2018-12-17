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
#define WJF_SENSORADC_MAX_INSTANCES           1
#define WJF_SENSORADC_PREARM_ALT_MAX_VOLT     5.0
#define WJF_SENSORADC_PREARM_ALT_MIN_VOLT     0.0
#define WJF_SENSORADC_USE_CH                  4

class WJF_SensorADC_Backend;

class WJF_SensorADC
{
    friend class WJF_SensorADC_Backend;

public:
    WJF_SensorADC(void);

    /* Do not allow copies */
    WJF_SensorADC(const WJF_SensorADC &other) = delete;
    WJF_SensorADC &operator=(const WJF_SensorADC&) = delete;

    // sensor driver types
    enum WJF_SensorADC_Type {
        WJF_SensorADC_TYPE_NONE    = 0,
        WJF_SensorADC_TYPE_ADS1015 = 1
    };

    enum WJF_SensorADC_Status {
        WJF_SensorADC_NotConnected = 0,
        WJF_SensorADC_NoData,
        WJF_SensorADC_OutOfRangeLow,
        WJF_SensorADC_OutOfRangeHigh,
        WJF_SensorADC_Good
    };

    // The WJF_SensorADC_State structure is filled in by the backend driver
    struct WJF_SensorADC_State {
        uint8_t                instance;    // the instance number of this WJF_SensorADC
        float                  voltage[OGR_SENSORTEMPMOTOR_USE_CH];  // voltage

        enum WJF_SensorADC_Status status;     // sensor status
        uint8_t                valid_count;   // number of consecutive valid readings (maxes out at 10)
        bool                   pre_arm_check;   // true if sensor has passed pre-arm checks
        float                  pre_arm_voltage_min;    // min temperature captured during pre-arm checks
        float                  pre_arm_voltage_max;    // max temperature captured during pre-arm checks

        AP_Int8  type;
        AP_Int8  address;
        AP_Float min_voltage[WJF_SENSORADC_USE_CH];
        AP_Float max_voltage[WJF_SENSORADC_USE_CH];
        AP_Int8  ch[WJF_SENSORADC_USE_CH]; //using ADC channel number
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

    WJF_SensorADC_Backend *get_backend(uint8_t id) const;

    /*
      returns true if pre-arm checks have passed for all temperature sensors
      these checks involve the user lifting or rotating the vehicle so that sensor readings between
      the min and 2m can be captured
     */
    bool pre_arm_check() const;


private:
    WJF_SensorADC_State state[WJF_SENSORADC_MAX_INSTANCES];
    WJF_SensorADC_Backend *drivers[WJF_SENSORADC_MAX_INSTANCES];
    uint8_t num_instances:1;

    void detect_instance(uint8_t instance);
    void update_instance(uint8_t instance);  

    bool _add_backend(WJF_SensorADC_Backend *driver);
};
