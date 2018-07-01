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
#include "OGR_SensorTempMotor_ADS1015_S8120C.h"

#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;

/*
   The constructor also initializes the sensor. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the sensor
*/
OGR_SensorTempMotor_ADS1015_S8120C::OGR_SensorTempMotor_ADS1015_S8120C(OGR_SensorTempMotor::OGR_SensorTempMotor_State &_state, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : OGR_SensorTempMotor_Backend(_state)
    , _dev(std::move(dev)) {}

/*
   detect if a sensor is connected. We'll detect by
   trying to take a reading on I2C. If we get a result the sensor is
   there.
*/
OGR_SensorTempMotor_Backend *OGR_SensorTempMotor_ADS1015_S8120C::detect(OGR_SensorTempMotor::OGR_SensorTempMotor_State &_state, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    OGR_SensorTempMotor_ADS1015_S8120C *sensor
        = new OGR_SensorTempMotor_ADS1015_S8120C(_state, std::move(dev));

    if (!sensor) {
        delete sensor;
        return nullptr;
    }

    if (sensor->_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        float temperature;
        float voltage;
        if (!sensor->get_reading(temperature, voltage)) {
            sensor->_dev->get_semaphore()->give();
            delete sensor;
            return nullptr;
        }
        sensor->_dev->get_semaphore()->give();
    }

    sensor->init();

    return sensor;
}

void OGR_SensorTempMotor_ADS1015_S8120C::init()
{
    // call timer() at 4Hz
    _dev->register_periodic_callback(250000,
                                     FUNCTOR_BIND_MEMBER(&OGR_SensorTempMotor_ADS1015_S8120C::timer, void));
}

// read - return last value measured by sensor
bool OGR_SensorTempMotor_ADS1015_S8120C::get_reading(float &temp, float &volt)
{
    be16_t val;
    uint16_t uval;
    int ival;

    if (state.address == 0) {
        return false;
    }

    if (state.ch > 3) {
        return false;
    }

  // Start with default values
  uint16_t config = ADS1015_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)
                    ADS1015_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
                    ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
                    ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
                    ADS1015_REG_CONFIG_DR_1600SPS   | // 1600 samples per second (default)
                    ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)

  // Set PGA/voltage range
  config |= ADS1015_REG_CONFIG_PGA_2_048V;

  // Set single-ended input channel
  switch (state.ch)
  {
    case (0):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_0;
      break;
    case (1):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_1;
      break;
    case (2):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_2;
      break;
    case (3):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_3;
      break;
  }

    bool ret;

  // Set 'start single-conversion' bit
  config |= ADS1015_REG_CONFIG_OS_SINGLE;

  // Write config register to the ADC
    uint8_t b[3] = { ADS1015_REG_POINTER_CONFIG, uint8_t(config>>8), uint8_t(config&0xff) };
    ret = _dev->transfer(b, 3, nullptr, 0);
    if (ret==false){
        return ret;
    }

  // Write config register to the ADC
    ret = _dev->transfer(b, 3, nullptr, 0);
    if (ret==false){
        return ret;
    }

  // Wait for the conversion to complete
  hal.scheduler->delay(1);

  // Read the conversion results
    ret = _dev->transfer(ADS1015_REG_POINTER_CONVERT, 1, (uint8_t *)&val, sizeof(val));
    if (ret) {
        // combine results and conversion
        uval = be16toh(val);
        // convert to 12-bit signed value.
        uval >>= 4;
        ival = (int)uval; // 
        if(uval & (0x8000 >> 4)) {
            ival -= 1 << 12; // when temperature is negative number
        }
//        volt = (float)ival*2.048/2048; // convert to voltage
        volt = (float)ival*0.001; // convert to voltage
//        temp = ((volt*1000) - 1705)/-8.2; // convert to celsius value
        temp = ((float)ival - 1705)/-8.2; // convert to celsius value
//        temp = 0.0005*(temp^2) - 0.0299*temp - 1.3426; // correct celsius value
    }

    return ret;
}

/*
   update the state of the sensor
*/
void OGR_SensorTempMotor_ADS1015_S8120C::update(void)
{
    // nothing to do - its all done in the timer()
}

void OGR_SensorTempMotor_ADS1015_S8120C::timer(void)
{
    if (get_reading(state.temperature, state.voltage)) {
        // update temp_valid state based on temperature measured
        update_status();
    } else {
        set_status(OGR_SensorTempMotor::OGR_SensorTempMotor_NoData);
    }
}
