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
#include "OGR_SensorTemp_ADT7410.h"

#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;

/*
   The constructor also initializes the sensor. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the sensor
*/
OGR_SensorTemp_ADT7410::OGR_SensorTemp_ADT7410(OGR_SensorTemp::OGR_SensorTemp_State &_state, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : OGR_SensorTemp_Backend(_state)
    , _dev(std::move(dev)) {}

/*
   detect if a sensor is connected. We'll detect by
   trying to take a reading on I2C. If we get a result the sensor is
   there.
*/
OGR_SensorTemp_Backend *OGR_SensorTemp_ADT7410::detect(OGR_SensorTemp::OGR_SensorTemp_State &_state, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    OGR_SensorTemp_ADT7410 *sensor
        = new OGR_SensorTemp_ADT7410(_state, std::move(dev));

    if (!sensor) {
        delete sensor;
        return nullptr;
    }

    if (sensor->_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        float temperature;
        if (!sensor->get_reading(temperature)) {
            sensor->_dev->get_semaphore()->give();
            delete sensor;
            return nullptr;
        }
        sensor->_dev->get_semaphore()->give();
    }

    sensor->init();

    return sensor;
}

void OGR_SensorTemp_ADT7410::init()
{
    // call timer() at 4Hz
    _dev->register_periodic_callback(250000,
                                     FUNCTOR_BIND_MEMBER(&OGR_SensorTemp_ADT7410::timer, void));
}

// read - return last value measured by sensor
bool OGR_SensorTemp_ADT7410::get_reading(float &reading_temp)
{
    be16_t val;
    uint16_t uval;
    int ival;

    if (state.address == 0) {
        return false;
    }

    // read the high and low byte temperature registers
    bool ret = _dev->read((uint8_t *) &val, sizeof(val));
    if (ret) {
        // combine results into distance
        uval = be16toh(val);
        uval >>= 3; // 13bit
        ival = (int)uval; // 
        if(uval & (0x8000 >> 3)) {
            ival = ival  - 8192; // when temperature is negative number
        }
        reading_temp = (float)ival / 16.0; // covert to celsius value
    }

    return ret;
}

/*
   update the state of the sensor
*/
void OGR_SensorTemp_ADT7410::update(void)
{
    // nothing to do - its all done in the timer()
}

void OGR_SensorTemp_ADT7410::timer(void)
{
    if (get_reading(state.temperature)) {
        // update temp_valid state based on temperature measured
        update_status();
    } else {
        set_status(OGR_SensorTemp::OGR_SensorTemp_NoData);
    }
}
