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
#include "OGR_SensorGas.h"

class OGR_SensorGas_Backend
{
public:
    // constructor. This incorporates initialisation as well.
	OGR_SensorGas_Backend(OGR_SensorGas::OGR_SensorGas_State &_state);

    // we declare a virtual destructor so that OGR_SensorGas drivers can
    // override with a custom destructor if need be
    virtual ~OGR_SensorGas_Backend(void) {}

    // update the state structure
    virtual void update() = 0;

    void update_pre_arm_check();

    uint8_t instance() const { return state.instance; }
/*
    float temperature() const { return state.temperature; }
    float voltage() const { return state.voltage; }
    float min_temperature() const { return state.min_temperature; }
    float max_temperature() const { return state.max_temperature; }
*/
    OGR_SensorGas::OGR_SensorGas_Status status() const {
        if (state.type == OGR_SensorGas::OGR_SensorGas_TYPE_NONE) {
            // turned off at runtime?
            return OGR_SensorGas::OGR_SensorGas_NotConnected;
        }
        return state.status;
    }
    OGR_SensorGas::OGR_SensorGas_Type type() const { return (OGR_SensorGas::OGR_SensorGas_Type)state.type.get(); }

    // true if sensor is returning data
    bool has_data() const {
        return ((state.status != OGR_SensorGas::OGR_SensorGas_NotConnected) &&
                (state.status != OGR_SensorGas::OGR_SensorGas_NoData));
    }

    // returns count of consecutive good readings
    uint8_t valid_count() const { return state.valid_count; }

    OGR_SensorGas::OGR_SensorGas_State &state;


protected:

    // update status based on distance measurement
    void update_status();

    // set status and update valid_count
    void set_status(OGR_SensorGas::OGR_SensorGas_Status status);

    // semaphore for access to shared frontend data
    AP_HAL::Semaphore *_sem;    
};
