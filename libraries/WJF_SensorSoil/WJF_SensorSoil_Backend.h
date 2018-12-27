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
#include "WJF_SensorSoil.h"

class WJF_SensorSoil_Backend
{
public:
    // constructor. This incorporates initialisation as well.
	WJF_SensorSoil_Backend(WJF_SensorSoil::WJF_SensorSoil_State &_state);

    // we declare a virtual destructor so that WJF_SensorSoil drivers can
    // override with a custom destructor if need be
    virtual ~WJF_SensorSoil_Backend(void) {}

    // update the state structure
    virtual void update() = 0;

    void update_pre_arm_check();

    uint8_t instance() const { return state.instance; }
    float temperature() const { return state.temperature; }
    float ph() const { return state.ph; }
    float ec() const { return state.ec; }
    float ec_phase() const { return state.ec_phase; }
    float min_temperature() const { return state.min_temperature; }
    float max_temperature() const { return state.max_temperature; }
    WJF_SensorSoil::WJF_SensorSoil_Status status() const {
        if (state.type == WJF_SensorSoil::WJF_SensorSoil_TYPE_NONE) {
            // turned off at runtime?
            return WJF_SensorSoil::WJF_SensorSoil_NotConnected;
        }
        return state.status;
    }
    WJF_SensorSoil::WJF_SensorSoil_Type type() const { return (WJF_SensorSoil::WJF_SensorSoil_Type)state.type.get(); }

    // true if sensor is returning data
    bool has_data() const {
        return ((state.status != WJF_SensorSoil::WJF_SensorSoil_NotConnected) &&
                (state.status != WJF_SensorSoil::WJF_SensorSoil_NoData));
    }

    // returns count of consecutive good readings
    uint8_t valid_count() const { return state.valid_count; }


protected:

    // update status based on distance measurement
    void update_status();

    // set status and update valid_count
    void set_status(WJF_SensorSoil::WJF_SensorSoil_Status status);

    WJF_SensorSoil::WJF_SensorSoil_State &state;

    // semaphore for access to shared frontend data
    AP_HAL::Semaphore *_sem;    
};
