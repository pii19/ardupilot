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
#include <AP_SerialManager/AP_SerialManager.h>

// Maximum number of temperature sensor instances available on this platform
#define WJF_SENSORSOIL_MAX_INSTANCES 1
#define WJF_SENSORSOIL_PREARM_ALT_MAX_TEMP           125.0
#define WJF_SENSORSOIL_PREARM_ALT_MIN_TEMP           -40.0

class WJF_SensorSoil_Backend;

class WJF_SensorSoil
{
    friend class WJF_SensorSoil_Backend;

public:
    WJF_SensorSoil(AP_SerialManager &_serial_manager);

    /* Do not allow copies */
    WJF_SensorSoil(const WJF_SensorSoil &other) = delete;
    WJF_SensorSoil &operator=(const WJF_SensorSoil&) = delete;

    // sensor driver types
    enum WJF_SensorSoil_Type {
        WJF_SensorSoil_TYPE_NONE   = 0,
        WJF_SensorSoil_TYPE_NJ1011 = 1
    };

    enum WJF_SensorSoil_Status {
        WJF_SensorSoil_NotConnected = 0,
        WJF_SensorSoil_NoData,
        WJF_SensorSoil_OutOfRangeLow,
        WJF_SensorSoil_OutOfRangeHigh,
        WJF_SensorSoil_Good
    };

    // The WJF_SensorSoil_State structure is filled in by the backend driver
    struct WJF_SensorSoil_State {
        uint8_t                instance;    // the instance number of this WJF_SensorSoil
        float                  temperature; // temperature: in celsius
        float                  ph; // pH
        float                  ec; // ec
        float                  ec_phase; // ec phase
                                            // if applicable, otherwise 0
        enum WJF_SensorSoil_Status status;     // sensor status
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

    WJF_SensorSoil_Backend *get_backend(uint8_t id) const;

    /*
      returns true if pre-arm checks have passed for all temperature sensors
      these checks involve the user lifting or rotating the vehicle so that sensor readings between
      the min and 2m can be captured
     */
    bool pre_arm_check() const;


private:
    WJF_SensorSoil_State state[WJF_SENSORSOIL_MAX_INSTANCES];
    WJF_SensorSoil_Backend *drivers[WJF_SENSORSOIL_MAX_INSTANCES];
    uint8_t num_instances:1;
    AP_SerialManager &serial_manager;

    void detect_instance(uint8_t instance);
    void update_instance(uint8_t instance);  

    bool _add_backend(WJF_SensorSoil_Backend *driver);
};
