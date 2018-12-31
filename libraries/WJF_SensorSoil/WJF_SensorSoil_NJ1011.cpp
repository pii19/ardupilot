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

#include <AP_HAL/AP_HAL.h>
#include "WJF_SensorSoil_NJ1011.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

/* 
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
WJF_SensorSoil_NJ1011::WJF_SensorSoil_NJ1011(WJF_SensorSoil::WJF_SensorSoil_State &_state,
                                                               AP_SerialManager &serial_manager) :
    WJF_SensorSoil_Backend(_state)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_None, 0);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_None, 0));
//        hal.scheduler->delay(5000);
//        write_command();
        last_reading_ms = AP_HAL::millis();
    }
}

/* 
   detect if a Lightware rangefinder is connected. We'll detect by
   trying to take a reading on Serial. If we get a result the sensor is
   there.
*/
bool WJF_SensorSoil_NJ1011::detect(AP_SerialManager &serial_manager)
{
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_None, 0) != nullptr;
}

// read - return last value measured by sensor
bool WJF_SensorSoil_NJ1011::get_reading()
{
    if (uart == nullptr) {
        return false;
    }

    // read last data on the receive buffer
    float sum = 0;
    bool fget = false;
    int16_t nbytes = uart->available();
    gcs().send_text(MAV_SEVERITY_CRITICAL, "serial available %d bytes.", nbytes);
    while (nbytes-- > 0) {
        char c = uart->read();
        if (c == '\n') {
            linebuf[linebuf_len] = 0;
            if (strcmp(linebuf, "OK") == 0) {	//WAKEUP�����ǂݎ̂�
                linebuf_len = 0;
                gcs().send_text(MAV_SEVERITY_CRITICAL, "recieve WAKEUP response.");
                write_measure();
//                continue;
            } else if (strlen(linebuf)>76) {	//MEASURE�����p�[�X
                gcs().send_text(MAV_SEVERITY_CRITICAL, "recieve MEASURE response.");
                int parse_count = 0;
                char parsebuf[64];
                int parsebuf_len = 0;
                for (int i=0; i<linebuf_len; i++) {
                    char pc = linebuf[i];
                    if (pc == ',') {
                        parsebuf[parsebuf_len] = 0;
                        switch (parse_count++) {
                            case 3: // temp
                                state.temperature = (float)atof(parsebuf);
                                break;
                            case 4: // pH
                                state.ph = (float)atof(parsebuf);
                                break;
                            case 6: // EC
                                state.ec = (float)atof(parsebuf);
                                break;
                            case 8: // EC phase
                                state.ec_phase = (float)atof(parsebuf);
                                fget = true;
                                break;
                            default:
                                break;
                        }
                        parsebuf_len = 0;
                    } else {
                        parsebuf[parsebuf_len++] = pc;
                    }
                }
            } else {	//�S�~����
            }
            linebuf_len = 0;
        } else {
            linebuf[linebuf_len++] = c;
            if (linebuf_len == sizeof(linebuf)) {
                // too long, discard the line
                linebuf_len = 0;
            }
        }
    }

    if (fget) {
        // we need to write command for next reading
        write_wakeup();
        fget = false;
        return true;
    }
    return false;
/*
// for MavLink debug
	state.temperature = (float)atof("+19.0");
	state.ph = (float)atof("+05.0");
	state.ec = (float)atof("+000.0400");
	state.ec_phase = (float)atof("-1.39452");
	return true;
*/
}

/* 
   update the state of the sensor
*/
void WJF_SensorSoil_NJ1011::write_wakeup(void)
{
    // send WAKEUP
    uart->write("WAKEUP");
    uart->write(0x0a);
    gcs().send_text(MAV_SEVERITY_CRITICAL, "send WAKEUP command.");
}

void WJF_SensorSoil_NJ1011::write_measure(void)
{
    // send MEASURE
    uart->write("MEASURE");
    uart->write(0x0a);
    gcs().send_text(MAV_SEVERITY_CRITICAL, "send MEASURE command.");
    last_reading_ms = AP_HAL::millis();
}

void WJF_SensorSoil_NJ1011::update(void)
{
    if (get_reading()) {
        // update range_valid state based on distance measured
        last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (AP_HAL::millis() - last_reading_ms > 15000) {	// timeout
        set_status(WJF_SensorSoil::WJF_SensorSoil_NoData);
        write_wakeup();
    }
}
