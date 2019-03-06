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
#include "WJF_SensorADC_ADS1015.h"

#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

extern const AP_HAL::HAL& hal;

/*
   The constructor also initializes the sensor. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the sensor
*/
WJF_SensorADC_ADS1015::WJF_SensorADC_ADS1015(WJF_SensorADC::WJF_SensorADC_State &_state, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : WJF_SensorADC_Backend(_state)
    , _dev(std::move(dev)) {}

/*
   detect if a sensor is connected. We'll detect by
   trying to take a reading on I2C. If we get a result the sensor is
   there.
*/
WJF_SensorADC_Backend *WJF_SensorADC_ADS1015::detect(WJF_SensorADC::WJF_SensorADC_State &_state, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    WJF_SensorADC_ADS1015 *sensor
        = new WJF_SensorADC_ADS1015(_state, std::move(dev));

    if (!sensor) {
        delete sensor;
        return nullptr;
    }

    if (sensor->_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        if (!sensor->get_reading()) {
            sensor->_dev->get_semaphore()->give();
            delete sensor;
            return nullptr;
        }
        sensor->_dev->get_semaphore()->give();
    }

    sensor->init();

    return sensor;
}

void WJF_SensorADC_ADS1015::init()
{
    // call timer() at 4Hz
    _dev->register_periodic_callback(250000,
                                     FUNCTOR_BIND_MEMBER(&WJF_SensorADC_ADS1015::timer, void));
}

// read - return last value measured by sensor
bool WJF_SensorADC_ADS1015::get_reading(void)
{
    be16_t val;
    int ival;
	bool ret=false;

    if (state.address == 0) {
        return ret;
    }

	for (uint8_t i=0; i<WJF_SENSORADC_USE_CH; i++) {
		if (state.ch[i] > 0) {
			if (get_ADS1015(state.ch[i]-1, val)) {
				if (get_ADS1015(state.ch[i]-1, val)) {
					ival = convert(val);
//        volt = (float)ival*2.048/2048; // convert to voltage
					state.voltage[i] = (float)ival*0.001*2; // convert to real scaling voltage
//        temp = ((volt*1000) - 1705)/-8.2; // convert to celsius value
//					state.temperature[i] = ((float)ival - 1705)/-8.2; // convert to celsius value
//        temp = 0.0005*(temp^2) - 0.0299*temp - 1.3426; // correct celsius value
					ret = true;
					// Wait for the next conversion
//					hal.scheduler->delay(1);
				}
			}
		}
	}
    return ret;
}

int WJF_SensorADC_ADS1015::convert(be16_t val)
{
    uint16_t uval;
    int ival;

        // combine results and conversion
        uval = be16toh(val);
        // convert to 12-bit signed value.
        uval >>= 4;
        ival = (int)uval; // 
        if(uval & (0x8000 >> 4)) {
            ival -= 1 << 12; // when temperature is negative number
        }

        return ival;
}

bool WJF_SensorADC_ADS1015::get_ADS1015(uint8_t ch, be16_t &val)
{
    bool ret=false;

  // Start with default values
  uint16_t config = ADS1015_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)
                    ADS1015_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
                    ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
                    ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
                    ADS1015_REG_CONFIG_DR_1600SPS; // 1600 samples per second (default)
//                    ADS1015_REG_CONFIG_DR_1600SPS   | // 1600 samples per second (default)
//                    ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)


  // Set single-ended input channel
  switch (ch)
  {
    case (0):
      config |= ADS1015_REG_CONFIG_PGA_2_048V;
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_0;
      break;
    case (1):
      config |= ADS1015_REG_CONFIG_PGA_2_048V;
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_1;
      break;
    case (2):
      config |= ADS1015_REG_CONFIG_PGA_4_096V;
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_2;
      break;
    case (3):
      config |= ADS1015_REG_CONFIG_PGA_1_024V;
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_3;
      break;
  }

  // Set 'start single-conversion' bit
//  config |= ADS1015_REG_CONFIG_OS_SINGLE;

  // Write config register to the ADC
    uint8_t b[3] = { ADS1015_REG_POINTER_CONFIG, uint8_t(config>>8), uint8_t(config&0xff) };
    ret = _dev->transfer(b, 3, nullptr, 0);
    if (ret==false){
        return ret;
    }

  // Wait for the conversion to complete
  hal.scheduler->delay(1);

  // Read the conversion results
	ret = _dev->transfer(ADS1015_REG_POINTER_CONVERT, 1, (uint8_t *)&val, sizeof(val));
	return ret;
}


/*
   update the state of the sensor
*/
void WJF_SensorADC_ADS1015::update(void)
{
    // nothing to do - its all done in the timer()
}

void WJF_SensorADC_ADS1015::timer(void)
{
    if (get_reading()) {
        // update temp_valid state based on temperature measured
        update_status();
    } else {
        set_status(WJF_SensorADC::WJF_SensorADC_NoData);
    }
}
