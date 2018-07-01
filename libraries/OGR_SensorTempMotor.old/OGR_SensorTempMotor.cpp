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

#include "OGR_SensorTempMotor.h"
#include "OGR_SensorTempMotor_ADS1015_S8120C.h"
#include <AP_BoardConfig/AP_BoardConfig.h>

extern const AP_HAL::HAL &hal;

// table of user settable parameters
const AP_Param::GroupInfo OGR_SensorTempMotor::var_info[] = {
    // @Param: TEMP_MOTOR1_TYPE
    // @DisplayName: Temperature sensor type
    // @Description: What type of sensor device that is connected
    // @Values: 0:None,1:ADS1015_S8120C
    // @User: Standard
    AP_GROUPINFO("1_TYPE", 0, OGR_SensorTempMotor, state[0].type, 1),

    // @Param: TEMP_MOTOR1_ADDR
    // @DisplayName: Bus address of sensor
    // @Description: This sets the I2C bus address of the sensor, where applicable. A value of 0 disables the sensor.
    // @Range: 0 127
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("1_ADDR", 1, OGR_SensorTempMotor, state[0].address, 0x4A),

    // @Param: TEMP_MOTOR1_CH
    // @DisplayName: Temperature sensor cahnnels number
    // @Description: Using channels number at I2C ADC
    // @Range: 1 4
    // @User: Standard
    AP_GROUPINFO("1_CH", 2, OGR_SensorTempMotor, state[0].ch, 0),

    // @Param: TEMP_MOTOR2_TYPE
    // @DisplayName: Temperature sensor type
    // @Description: What type of sensor device that is connected
    // @Values: 0:None,1:ADS1015_S8120C
    // @User: Standard
    AP_GROUPINFO("2_TYPE", 3, OGR_SensorTempMotor, state[1].type, 0),

    // @Param: TEMP_MOTOR2_ADDR
    // @DisplayName: Bus address of sensor
    // @Description: This sets the I2C bus address of the sensor, where applicable. A value of 0 disables the sensor.
    // @Range: 0 127
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("2_ADDR", 4, OGR_SensorTempMotor, state[1].address, 0x4A),

    // @Param: TEMP_MOTOR2_CH
    // @DisplayName: Temperature sensor cahnnels number
    // @Description: Using channels number at I2C ADC
    // @Range: 1 4
    // @User: Standard
    AP_GROUPINFO("2_CH", 5, OGR_SensorTempMotor, state[1].ch, 1),

    // @Param: TEMP_MOTOR3_TYPE
    // @DisplayName: Temperature sensor type
    // @Description: What type of sensor device that is connected
    // @Values: 0:None,1:ADS1015_S8120C
    // @User: Standard
    AP_GROUPINFO("3_TYPE", 6, OGR_SensorTempMotor, state[2].type, 0),

    // @Param: TEMP_MOTOR3_ADDR
    // @DisplayName: Bus address of sensor
    // @Description: This sets the I2C bus address of the sensor, where applicable. A value of 0 disables the sensor.
    // @Range: 0 127
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("3_ADDR", 7, OGR_SensorTempMotor, state[2].address, 0x4A),

    // @Param: TEMP_MOTOR3_CH
    // @DisplayName: Temperature sensor cahnnels number
    // @Description: Using channels number at I2C ADC
    // @Range: 1 4
    // @User: Standard
    AP_GROUPINFO("3_CH", 8, OGR_SensorTempMotor, state[2].ch, 2),

    // @Param: TEMP_MOTOR4_TYPE
    // @DisplayName: Temperature sensor type
    // @Description: What type of sensor device that is connected
    // @Values: 0:None,1:ADS1015_S8120C
    // @User: Standard
    AP_GROUPINFO("4_TYPE", 9, OGR_SensorTempMotor, state[3].type, 1),

    // @Param: TEMP_MOTOR4_ADDR
    // @DisplayName: Bus address of sensor
    // @Description: This sets the I2C bus address of the sensor, where applicable. A value of 0 disables the sensor.
    // @Range: 0 127
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("4_ADDR", 10, OGR_SensorTempMotor, state[3].address, 0x4A),

    // @Param: TEMP_MOTOR4_CH
    // @DisplayName: Temperature sensor cahnnels number
    // @Description: Using channels number at I2C ADC
    // @Range: 1 4
    // @User: Standard
    AP_GROUPINFO("4_CH", 11, OGR_SensorTempMotor, state[3].ch, 3),

    AP_GROUPEND
};

OGR_SensorTempMotor::OGR_SensorTempMotor() :
    num_instances(0)
{
    AP_Param::setup_object_defaults(this, var_info);
}

/*
  initialise the OGR SensorTemp class. We do detection of attached sensors
  here. For now we won't allow for hot-plugging of several sensors.
*/
void OGR_SensorTempMotor::init(void)
{
    if (num_instances != 0) {
        // init called a 2nd time?
        return;
    }
    for (uint8_t i=0; i<OGR_SENSORTEMPMOTOR_MAX_INSTANCES; i++) {
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
        state[i].min_temperature = OGR_SENSORTEMPMOTOR_PREARM_ALT_MIN_TEMP;
        state[i].max_temperature = OGR_SENSORTEMPMOTOR_PREARM_ALT_MAX_TEMP;

        // initialise status
        state[i].status = OGR_SensorTempMotor_NotConnected;
        state[i].valid_count = 0;
    }
}

/*
  update state for all instances. This should be called at
  around 10Hz by main loop
 */
void OGR_SensorTempMotor::update(void)
{
    for (uint8_t i=0; i<OGR_SENSORTEMPMOTOR_MAX_INSTANCES; i++) {
        if (drivers[i] != nullptr) {
            if (state[i].type == OGR_SensorTempMotor_TYPE_NONE) {
                // allow user to disable a sensor at runtime
                state[i].status = OGR_SensorTempMotor_NotConnected;
                state[i].valid_count = 0;
                continue;
            }
            drivers[i]->update();
            drivers[i]->update_pre_arm_check();
        }
    }
}

bool OGR_SensorTempMotor::_add_backend(OGR_SensorTempMotor_Backend *backend)
{
    if (!backend) {
        return false;
    }
    if (num_instances == OGR_SENSORTEMPMOTOR_MAX_INSTANCES) {
        AP_HAL::panic("Too many OGR temperature sensor backends");
    }

    drivers[num_instances++] = backend;
    return true;
}

/*
  detect if an instance of a sensor is connected. 
 */
void OGR_SensorTempMotor::detect_instance(uint8_t instance)
{
    enum OGR_SensorTempMotor_Type _type = (enum OGR_SensorTempMotor_Type)state[instance].type.get();
    switch (_type) {
    case OGR_SensorTempMotor_TYPE_ADS1015_S8120C:
        if (state[instance].address) {
            if (!_add_backend(OGR_SensorTempMotor_ADS1015_S8120C::detect(state[instance], hal.i2c_mgr->get_device(1, state[instance].address)))) {
                _add_backend(OGR_SensorTempMotor_ADS1015_S8120C::detect(state[instance], hal.i2c_mgr->get_device(0, state[instance].address)));
            }
        }
        break;
    default:
        break;
    }
}

OGR_SensorTempMotor_Backend *OGR_SensorTempMotor::get_backend(uint8_t id) const {
    if (id >= num_instances) {
        return nullptr;
    }
    if (drivers[id] != nullptr) {
        if (drivers[id]->type() == OGR_SensorTempMotor_TYPE_NONE) {
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
bool OGR_SensorTempMotor::pre_arm_check() const
{
    for (uint8_t i=0; i<num_instances; i++) {
        // if driver is valid but pre_arm_check is false, return false
        if ((drivers[i] != nullptr) && (state[i].type != OGR_SensorTempMotor_TYPE_NONE) && !state[i].pre_arm_check) {
            return false;
        }
    }
    return true;
}
