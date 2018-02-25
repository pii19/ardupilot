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

#include "OGR_SensorTemp.h"
#include "OGR_SensorTemp_ADT7410.h"
#include <AP_BoardConfig/AP_BoardConfig.h>

extern const AP_HAL::HAL &hal;

// table of user settable parameters
const AP_Param::GroupInfo OGR_SensorTemp::var_info[] = {
    // @Param: ADT7410_1_ADDR
    // @DisplayName: Bus address of sensor
    // @Description: This sets the bus address of the 1st ADT7410 sensor, where applicable. A value of 0 disables the sensor.
    // @Range: 0 127
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ADT7410_1_ADDR", 1, OGR_SensorTemp, state[0].address, 0),

    AP_GROUPEND
};

OGR_SensorTemp::OGR_SensorTemp() :
    num_instances(0)
{
    AP_Param::setup_object_defaults(this, var_info);
}

/*
  initialise the OGR SensorTemp class. We do detection of attached sensors
  here. For now we won't allow for hot-plugging of several sensors.
*/
void OGR_SensorTemp::init(void)
{
    if (num_instances != 0) {
        // init called a 2nd time?
        return;
    }
    for (uint8_t i=0; i<OGR_SENSORTEMP_MAX_INSTANCES; i++) {
        state[i].address = 0x48;
        state[i].type = OGR_SensorTemp_TYPE_ADT7410;
        detect_instance(i);
        if (drivers[i] != nullptr) {
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy)
            num_instances = i+1;
        }
        // initialise pre-arm check variables
        state[i].pre_arm_check = false;
        state[i].pre_arm_temperature_min = 9999;  // initialise to an arbitrary large value
        state[i].pre_arm_temperature_max = -10000;  // initialise to an arbitrary small value

        // initialise status
        state[i].status = OGR_SensorTemp_NotConnected;
        state[i].valid_count = 0;
    }
}

/*
  update state for all instances. This should be called at
  around 10Hz by main loop
 */
void OGR_SensorTemp::update(void)
{
    for (uint8_t i=0; i<num_instances; i++) {
        if (drivers[i] != nullptr) {
            if (state[i].type == OGR_SensorTemp_TYPE_NONE) {
                // allow user to disable a sensor at runtime
                state[i].status = OGR_SensorTemp_NotConnected;
                state[i].valid_count = 0;
                continue;
            }
            drivers[i]->update();
            drivers[i]->update_pre_arm_check();
        }
    }
}

bool OGR_SensorTemp::_add_backend(OGR_SensorTemp_Backend *backend)
{
    if (!backend) {
        return false;
    }
    if (num_instances == OGR_SENSORTEMP_MAX_INSTANCES) {
        AP_HAL::panic("Too many OGR temperature sensor backends");
    }

    drivers[num_instances++] = backend;
    return true;
}

/*
  detect if an instance of a sensor is connected. 
 */
void OGR_SensorTemp::detect_instance(uint8_t instance)
{
    enum OGR_SensorTemp_Type _type = (enum OGR_SensorTemp_Type)state[instance].type.get();
    switch (_type) {
    case OGR_SensorTemp_TYPE_ADT7410:
        if (state[instance].address) {
            if (!_add_backend(OGR_SensorTemp_ADT7410::detect(state[instance], hal.i2c_mgr->get_device(1, state[instance].address)))) {
                _add_backend(OGR_SensorTemp_ADT7410::detect(state[instance], hal.i2c_mgr->get_device(0, state[instance].address)));
            }
        }
        break;
    default:
        break;
    }
}

OGR_SensorTemp_Backend *OGR_SensorTemp::get_backend(uint8_t id) const {
    if (id >= num_instances) {
        return nullptr;
    }
    if (drivers[id] != nullptr) {
        if (drivers[id]->type() == OGR_SensorTemp_TYPE_NONE) {
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
bool OGR_SensorTemp::pre_arm_check() const
{
    for (uint8_t i=0; i<num_instances; i++) {
        // if driver is valid but pre_arm_check is false, return false
        if ((drivers[i] != nullptr) && (state[i].type != OGR_SensorTemp_TYPE_NONE) && !state[i].pre_arm_check) {
            return false;
        }
    }
    return true;
}
