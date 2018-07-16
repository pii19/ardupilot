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
#include "OGR_SensorGas.h"
#include "OGR_SensorGas_Backend.h"

extern const AP_HAL::HAL& hal;

/*
  base class constructor. 
  This incorporates initialisation as well.
*/
OGR_SensorGas_Backend::OGR_SensorGas_Backend(OGR_SensorGas::OGR_SensorGas_State &_state) :
        state(_state)
{
    _sem = hal.util->new_semaphore();    
}

// update status based on concentration measurement
void OGR_SensorGas_Backend::update_status()
{
/*
    // check concentration
    if (state.concentration > state.max_concentration) {
        set_status(OGR_SensorGas::OGR_SensorGas_OutOfRangeHigh);
    } else if ((int16_t)state.concentration < state.min_concentration) {
        set_status(OGR_SensorGas::OGR_SensorGas_OutOfRangeLow);
    } else {
        set_status(OGR_SensorGas::OGR_SensorGas_Good);
    }
*/
}

// set status and update valid count
void OGR_SensorGas_Backend::set_status(OGR_SensorGas::OGR_SensorGas_Status _status)
{
    state.status = _status;

    // update valid count
    if (_status == OGR_SensorGas::OGR_SensorGas_Good) {
        if (state.valid_count < 10) {
            state.valid_count++;
        }
    } else {
        state.valid_count = 0;
    }
}

/*
  set pre-arm checks to passed if the gas sensor has been exercised through a reasonable range of movement
      max concentration sensed is at least 50cm > min concentration sensed
      max concentration < 100 c
      min concentration sensed is within 10cm of ground clearance or sensor's concentration
 */
void OGR_SensorGas_Backend::update_pre_arm_check()
{/*
    // return immediately if already passed or no sensor data
    if (state.pre_arm_check || state.status == OGR_SensorGas::OGR_SensorGas_NotConnected || state.status == OGR_SensorGas::OGR_SensorGas_NoData) {
        return;
    }

    // update min, max captured concentrations
    state.pre_arm_concentration_min = MIN(state.concentration, state.pre_arm_concentration_min);
    state.pre_arm_concentration_max = MAX(state.concentration, state.pre_arm_concentration_max);

    // Check that the range finder has been exercised through a realistic range of movement
    if (
         (state.pre_arm_concentration_max < OGR_SENSORTEMPMOTOR_PREARM_ALT_MAX_TEMP) &&
         (state.pre_arm_concentration_min > OGR_SENSORTEMPMOTOR_PREARM_ALT_MIN_TEMP)
         ) {
        state.pre_arm_check = true;
    }
*/
}
