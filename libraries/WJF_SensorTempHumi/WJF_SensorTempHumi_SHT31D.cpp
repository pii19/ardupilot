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
#include "WJF_SensorTempHumi_SHT31D.h"

#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

/*
   The constructor also initializes the sensor. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the sensor
*/
WJF_SensorTempHumi_SHT31D::WJF_SensorTempHumi_SHT31D(WJF_SensorTempHumi::WJF_SensorTempHumi_State &_state, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : WJF_SensorTempHumi_Backend(_state)
    , _dev(std::move(dev)) {}

/*
   detect if a sensor is connected. We'll detect by
   trying to take a reading on I2C. If we get a result the sensor is
   there.
*/
WJF_SensorTempHumi_Backend *WJF_SensorTempHumi_SHT31D::detect(WJF_SensorTempHumi::WJF_SensorTempHumi_State &_state, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    WJF_SensorTempHumi_SHT31D *sensor
        = new WJF_SensorTempHumi_SHT31D(_state, std::move(dev));

    if (!sensor) {
        delete sensor;
        return nullptr;
    }

    if (sensor->_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        if (!sensor->get_reading()) {
            sensor->_dev->get_semaphore()->give();
            delete sensor;
            gcs().send_text(MAV_SEVERITY_INFO, "SHT31D is not detected");
            return nullptr;
        }
        sensor->_dev->get_semaphore()->give();
    }

    sensor->init();

    gcs().send_text(MAV_SEVERITY_INFO, "SHT31D is detected");
    return sensor;
}

void WJF_SensorTempHumi_SHT31D::init()
{
    // call timer() at 4Hz
    _dev->register_periodic_callback(250000,
                                     FUNCTOR_BIND_MEMBER(&WJF_SensorTempHumi_SHT31D::timer, void));
}

// read - return last value measured by sensor
bool WJF_SensorTempHumi_SHT31D::get_reading()
{
	SHT31D data;

    if (state.address == 0) {
        return false;
    }

    data = readTempAndHumidityPolling(SHT3XD_REPEATABILITY_MEDIUM, 50);
    if (data.error != SHT3XD_NO_ERROR ) {
        gcs().send_text(MAV_SEVERITY_INFO, "SHT31D error 1st");
        data = readTempAndHumidityPolling(SHT3XD_REPEATABILITY_LOW, 50);
        if (data.error != SHT3XD_NO_ERROR ) {
            gcs().send_text(MAV_SEVERITY_INFO, "SHT31D error 2nd");
            return false;
        }
    }
    state.temperature = data.t;
    state.humidity = data.rh;
    return true;
}

/*
   update the state of the sensor
*/
void WJF_SensorTempHumi_SHT31D::update(void)
{
    // nothing to do - its all done in the timer()
}

void WJF_SensorTempHumi_SHT31D::timer(void)
{
    if (get_reading()) {
        // update temp_valid state based on temperature measured
        update_status();
    } else {
        set_status(WJF_SensorTempHumi::WJF_SensorTempHumi_NoData);
    }
}




SHT31D WJF_SensorTempHumi_SHT31D::readTempAndHumidityPolling(SHT31D_Repeatability repeatability, uint8_t timeout)
{
	SHT31D_ErrorCode error = SHT3XD_NO_ERROR;

	switch (repeatability)
	{
	case SHT3XD_REPEATABILITY_LOW:
//		error = writeCommand(SHT3XD_CMD_POLLING_L);
		error = writeCommand(SHT3XD_CMD_CLOCK_STRETCH_L);
		break;
	case SHT3XD_REPEATABILITY_MEDIUM:
//		error = writeCommand(SHT3XD_CMD_POLLING_M);
		error = writeCommand(SHT3XD_CMD_CLOCK_STRETCH_M);
		break;
	case SHT3XD_REPEATABILITY_HIGH:
//		error = writeCommand(SHT3XD_CMD_POLLING_H);
		error = writeCommand(SHT3XD_CMD_CLOCK_STRETCH_H);
		break;
	default:
		error = SHT3XD_PARAM_WRONG_REPEATABILITY;
		break;
	}

//	hal.scheduler->delay(5);

	if (error == SHT3XD_NO_ERROR) {
		return readTemperatureAndHumidity();
	} else {
		return returnError(error);
	}

}

SHT31D_ErrorCode WJF_SensorTempHumi_SHT31D::writeCommand(SHT31D_Commands command)
{
	bool ret;
	SHT31D_ErrorCode error = SHT3XD_NO_ERROR;

    uint8_t b[2] = { uint8_t(command>>8), uint8_t(command&0xff) };
    ret = _dev->transfer(b, 2, nullptr, 0);
    if (ret==false){
        error = SHT3XD_WIRE_I2C_UNKNOW_ERROR;
    }
	return error;
}

SHT31D WJF_SensorTempHumi_SHT31D::readTemperatureAndHumidity()
{
	SHT31D result;

//	SHT31D_ErrorCode error = SHT3XD_NO_ERROR;
//	uint16_t data[2];
	uint8_t	buf[6];
//	int counter = 0;
/*
    for (counter = 0; counter < 2; counter++) {
		_dev->transfer(nullptr, 0, (uint8_t *)&buf, 3);
		data[counter] = (buf[0] << 8) | buf[1];
	}

	if (error == SHT3XD_NO_ERROR) {
		result.t = calculateTemperature(data[0]);
		result.rh = calculateHumidity(data[1]);
	}
*/
    bool ret = _dev->transfer(nullptr, 0, (uint8_t *)&buf, 6);
    if (ret) {
		result.t = calculateTemperature(buf[0]<<8|buf[1]);
		result.rh = calculateHumidity(buf[3]<<8|buf[4]);
        result.error = SHT3XD_NO_ERROR;
    } else {
    	result.t = 0;
	    result.rh = 0;
    	result.error = SHT3XD_WIRE_I2C_UNKNOW_ERROR;
    }

	return result;
}

float WJF_SensorTempHumi_SHT31D::calculateTemperature(uint16_t rawValue)
{
	return 175.0f * (float)rawValue / 65535.0f - 45.0f;
}

float WJF_SensorTempHumi_SHT31D::calculateHumidity(uint16_t rawValue)
{
	return 100.0f * rawValue / 65535.0f;
}

SHT31D WJF_SensorTempHumi_SHT31D::returnError(SHT31D_ErrorCode error) {
	SHT31D result;
	result.t = 0;
	result.rh = 0;
	result.error = error;
	return result;
}
