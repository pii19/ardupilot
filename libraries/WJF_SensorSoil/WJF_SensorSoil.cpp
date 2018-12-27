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

#include "WJF_SensorSoil.h"
#include "WJF_SensorSoil_NJ1011.h"
#include <AP_BoardConfig/AP_BoardConfig.h>

extern const AP_HAL::HAL &hal;

// table of user settable parameters
const AP_Param::GroupInfo WJF_SensorSoil::var_info[] = {
    // @Param: SOIL_TYPE
    // @DisplayName: Soil sensor type
    // @Description: What type of sensor device that is connected
    // @Values: 0:None,1:NJ1011
    // @User: Standard
    AP_GROUPINFO("_TYPE", 0, WJF_SensorSoil, state[0].type, 1),

    AP_GROUPEND
};

WJF_SensorSoil::WJF_SensorSoil(AP_SerialManager &_serial_manager) :
    num_instances(0),
    serial_manager(_serial_manager)
{
    AP_Param::setup_object_defaults(this, var_info);
}

/*
  initialise the WJF SensorSoil class. We do detection of attached sensors
  here. For now we won't allow for hot-plugging of several sensors.
*/
void WJF_SensorSoil::init(void)
{
    if (num_instances != 0) {
        // init called a 2nd time?
        return;
    }
    for (uint8_t i=0; i<WJF_SENSORSOIL_MAX_INSTANCES; i++) {
        detect_instance(i);
        if (drivers[i] != nullptr) {
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy)
        }
        num_instances = i+1;
        // initialise pre-arm check variables
        state[i].pre_arm_check = false;
        state[i].pre_arm_temperature_min = 9999;  // initialise to an arbitrary large value
        state[i].pre_arm_temperature_max = -10000;  // initialise to an arbitrary small value

        // initialise min/max temperature variables
        state[i].min_temperature = WJF_SENSORSOIL_PREARM_ALT_MIN_TEMP;
        state[i].max_temperature = WJF_SENSORSOIL_PREARM_ALT_MAX_TEMP;

        // initialise status
        state[i].status = WJF_SensorSoil_NotConnected;
        state[i].valid_count = 0;
    }
}

/*
  update state for all instances. This should be called at
  around 10Hz by main loop
 */
void WJF_SensorSoil::update(void)
{
    for (uint8_t i=0; i<WJF_SENSORSOIL_MAX_INSTANCES; i++) {
        if (drivers[i] != nullptr) {
            if (state[i].type == WJF_SensorSoil_TYPE_NONE) {
                // allow user to disable a sensor at runtime
                state[i].status = WJF_SensorSoil_NotConnected;
                state[i].valid_count = 0;
                continue;
            }
            drivers[i]->update();
            drivers[i]->update_pre_arm_check();
        }
    }
}

bool WJF_SensorSoil::_add_backend(WJF_SensorSoil_Backend *backend)
{
    if (!backend) {
        return false;
    }
    if (num_instances == WJF_SENSORSOIL_MAX_INSTANCES) {
        AP_HAL::panic("Too many WJF soil sensor backends");
    }

    drivers[num_instances++] = backend;
    return true;
}

/*
  detect if an instance of a sensor is connected. 
 */
void WJF_SensorSoil::detect_instance(uint8_t instance)
{
    enum WJF_SensorSoil_Type _type = (enum WJF_SensorSoil_Type)state[instance].type.get();
    switch (_type) {
    case WJF_SensorSoil_TYPE_NJ1011:
        if (WJF_SensorSoil_NJ1011::detect(serial_manager)) {
            state[instance].instance = instance;
            drivers[instance] = new WJF_SensorSoil_NJ1011(state[instance], serial_manager);
        }
        break;
    default:
        break;
    }
}

WJF_SensorSoil_Backend *WJF_SensorSoil::get_backend(uint8_t id) const {
    if (id >= num_instances) {
        return nullptr;
    }
    if (drivers[id] != nullptr) {
        if (drivers[id]->type() == WJF_SensorSoil_TYPE_NONE) {
            // pretend it isn't here; disabled at runtime?
            return nullptr;
        }
    }
    return drivers[id];
};

/*
  returns true if pre-arm checks have passed for all sensors
  these checks involve the user lifting or rotating the vehicle so that sensor readings between
  the min and 2m can be captured
 */
bool WJF_SensorSoil::pre_arm_check() const
{
    for (uint8_t i=0; i<num_instances; i++) {
        // if driver is valid but pre_arm_check is false, return false
        if ((drivers[i] != nullptr) && (state[i].type != WJF_SensorSoil_TYPE_NONE) && !state[i].pre_arm_check) {
            return false;
        }
    }
    return true;
}
