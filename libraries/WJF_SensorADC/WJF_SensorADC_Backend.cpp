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

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "WJF_SensorADC.h"
#include "WJF_SensorADC_Backend.h"

extern const AP_HAL::HAL& hal;

/*
  base class constructor. 
  This incorporates initialisation as well.
*/
WJF_SensorADC_Backend::WJF_SensorADC_Backend(WJF_SensorADC::WJF_SensorADC_State &_state) :
        state(_state)
{
    _sem = hal.util->new_semaphore();    
}

// update status based on temperature measurement
void WJF_SensorADC_Backend::update_status()
{

    // check voltage
    if (state.voltage > state.max_voltage) {
        set_status(WJF_SensorADC::WJF_SensorADC_OutOfRangeHigh);
    } else if ((int16_t)state.voltage < state.min_voltage) {
        set_status(WJF_SensorADC::WJF_SensorADC_OutOfRangeLow);
    } else {
        set_status(WJF_SensorADC::WJF_SensorADC_Good);
    }

}

// set status and update valid count
void WJF_SensorADC_Backend::set_status(WJF_SensorADC::WJF_SensorADC_Status _status)
{
    state.status = _status;

    // update valid count
    if (_status == WJF_SensorADC::WJF_SensorADC_Good) {
        if (state.valid_count < 10) {
            state.valid_count++;
        }
    } else {
        state.valid_count = 0;
    }
}

/*
  set pre-arm checks to passed if the temperature sensor has been exercised through a reasonable range of movement
      max temperature sensed is at least 50cm > min temperature sensed
      max temperature < 100 c
      min temperature sensed is within 10cm of ground clearance or sensor's temperature
 */
void WJF_SensorADC_Backend::update_pre_arm_check()
{/*
    // return immediately if already passed or no sensor data
    if (state.pre_arm_check || state.status == WJF_SensorADC::WJF_SensorADC_NotConnected || state.status == WJF_SensorADC::WJF_SensorADC_NoData) {
        return;
    }

    // update min, max captured temperatures
    state.pre_arm_temperature_min = MIN(state.temperature, state.pre_arm_temperature_min);
    state.pre_arm_temperature_max = MAX(state.temperature, state.pre_arm_temperature_max);

    // Check that the range finder has been exercised through a realistic range of movement
    if (
         (state.pre_arm_temperature_max < OGR_SENSORTEMPMOTOR_PREARM_ALT_MAX_TEMP) &&
         (state.pre_arm_temperature_min > OGR_SENSORTEMPMOTOR_PREARM_ALT_MIN_TEMP)
         ) {
        state.pre_arm_check = true;
    }
*/
}
