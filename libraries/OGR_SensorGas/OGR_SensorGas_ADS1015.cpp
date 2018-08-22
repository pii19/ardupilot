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
#include "OGR_SensorGas_ADS1015.h"

#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <unistd.h> //gas
extern const AP_HAL::HAL& hal;

/*
   The constructor also initializes the sensor. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the sensor
*/
OGR_SensorGas_ADS1015::OGR_SensorGas_ADS1015(OGR_SensorGas::OGR_SensorGas_State &_state, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : OGR_SensorGas_Backend(_state)
    , _dev(std::move(dev)) {}

/*
   detect if a sensor is connected. We'll detect by
   trying to take a reading on I2C. If we get a result the sensor is
   there.
*/
OGR_SensorGas_Backend *OGR_SensorGas_ADS1015::detect(OGR_SensorGas::OGR_SensorGas_State &_state, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    OGR_SensorGas_ADS1015 *sensor
        = new OGR_SensorGas_ADS1015(_state, std::move(dev));

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

void OGR_SensorGas_ADS1015::init()
{
    // call timer() at 4Hz
    _dev->register_periodic_callback(250000,
                                     FUNCTOR_BIND_MEMBER(&OGR_SensorGas_ADS1015::timer, void));
}

// read - return last value measured by sensor
bool OGR_SensorGas_ADS1015::get_reading(void)
{
    be16_t val;
    int ival;
	bool ret=false;

    //tgs2602 Ro,Rl,Vout,Vc set
    //float Ro = 99.9;       //Ro test
    //float Rl = 47000.0;    //Rl 47K ohm
    //float Vout = 0.0;
    //float Vc = 5.0;
    //float Rs = 0.0;

    //float Ro_init = 10000;  //10k ohm
    /*
    float H2S_Curve[2] = {0.05566582614,-2.954075758}; //TGS2602     (0.8,0.1) (0.4,1) (0.25,3)
    float SmokeCurve[2]   =  {3426.376355, -2.225037973};  //MQ2 smoke

    float Ro = 0.0;//2.511;       //TGS2602 0.05 this has to be tuned 10K Ohm
    float Rl_tg = 0.893;             //TGmq136S2602 Gas Sensor V1.3 auto-ctrl.com
    float Rl_mq2 = 2.897;    //MQ2     Elecfreacks Octopus 
    float val_sens = 0;          // variable to store the value coming from the sensor
    int data_count = 50;
    int i_num = 0;
    double ans = 0;    //sens data
    long data = 0;  //data calculation
    float Rs = 0.0;
    */

    if (state.address == 0) {
        return ret;
    }

    uint8_t i=0;
	if (state.ch[i] > 0) {
		if (get_ADS1015(state.ch[i]-1, val)) {
			if (get_ADS1015(state.ch[i]-1, val)) {
				ival = convert(val);
				state.voltage[i] = (float)ival*0.002; // convert to voltage
                //combustibility gas
                //set Ro
            /*    for (i_num = 0 ; i < data_count ; i_num++){
                    get_ADS1015(state.ch[i]-1, val);
                    ival = convert(val);
                    ival = ival * 0.002;
                    data = (ival - 0) * (1023 - 0) / (5 - 0) + 0;   //0-5 -> 0-1023
                    val_sens += (long)((long)(1024*1000*(long)Rl_mq2)/data-(long)Rl_mq2);
                    usleep(600000);
                }
                Ro = (float)((long)val_sens*exp(((float)(log(SmokeCurve[0]/10))/SmokeCurve[1])));
				
                //set Rs
                for (i_num = 0 ; i < data_count ; i_num++){
                    get_ADS1015(state.ch[i]-1, val);
                    ival = convert(val);
                    ival = ival * 0.002;
                    data = (ival - 0) * (1023 - 0) / (5 - 0) + 0;   //0-5 -> 0-1023
                    val_sens += (long)((long)(1024*1000*(long)Rl_mq2)/data-(long)Rl_mq2);
                    usleep(600000);
                }
                Rs = data / data_count;
                
                //<3.5 smoke true  >3.5 smoke false
                if((Rs/Ro) < 3.5){
                    ans = 999;
                }else{
                    ans = 0;
                }
                
                state.concentration[i] = ans;
            */
                //combustibility gas
				state.concentration[i] = ((float)ival - 1705)/-8.2; // convert to celsius value
                state.concentration[i] =111;              
                ret = true;
				// Wait for the next conversion
			}
		}
	}
    
    i=1;
    if (state.ch[i] > 0) {
		if (get_ADS1015(state.ch[i]-1, val)) {
			if (get_ADS1015(state.ch[i]-1, val)) {
				ival = convert(val);
				state.voltage[i] = (float)ival*0.002; // convert to voltage
                //H2S gas
                //set Ro
            /*    for (i_num = 0 ; i < data_count ; i_num++){
                    get_ADS1015(state.ch[i]-1, val);
                    ival = convert(val);
                    ival = ival * 0.002;
                    data = (ival - 0) * (1023 - 0) / (5 - 0) + 0;   //0-5 -> 0-1023
                    val_sens += (long)((long)(1024*1000*(long)Rl_tg)/data-(long)Rl_tg);
                    usleep(600000);
                }
                Ro = (float)((long)val_sens*exp(((float)(log(H2S_Curve[0]/1))/H2S_Curve[1])));

                //set Rs
                for (i_num = 0 ; i < data_count ; i_num++){
                    get_ADS1015(state.ch[i]-1, val);
                    ival = convert(val);
                    ival = ival * 0.002;
                    data = (ival - 0) * (1023 - 0) / (5 - 0) + 0;   //0-5 -> 0-1023
                    val_sens += (long)((long)(1024*1000*(long)Rl_tg)/data-(long)Rl_tg);
                    usleep(600000);
                }
                Rs = data / data_count;

                //set ans
                ans = (double)(H2S_Curve[0] * (float)pow((Rs/Ro),H2S_Curve[1]));
                state.concentration[i] = ans;
            */
				state.concentration[i] = ((float)ival - 1705)/-8.2; // convert to celsius value
                state.concentration[i] =333;
				ret = true;
				// Wait for the next conversion
			}
		}
	}

    i=2;
    if (state.ch[i] > 0) {
		if (get_ADS1015(state.ch[i]-1, val)) {
			if (get_ADS1015(state.ch[i]-1, val)) {
	    		ival = convert(val);
				state.voltage[i] = (float)ival*0.002; // convert to voltage
                //CO gans
				state.concentration[i] = ((float)ival - 2)/-0.0025; // convert to celsius value
                //↑定義元確認
                state.concentration[i] =999;
				ret = true;
				// Wait for the next conversion
			}
		}
	}
    return ret;
}

int OGR_SensorGas_ADS1015::convert(be16_t val)
{
    uint16_t uval;
    int ival;

        // combine results and conversion
        uval = be16toh(val);
        // convert to 12-bit signed value.
        uval >>= 4;
        ival = (int)uval; // 
        if(uval & (0x8000 >> 4)) {
            ival -= 1 << 12; // when concentration is negative number
        }

        return ival;
}

bool OGR_SensorGas_ADS1015::get_ADS1015(uint8_t ch, be16_t &val)
{
    bool ret=false;

  // Start with default values
  uint16_t config = ADS1015_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)
                    ADS1015_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
                    ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
                    ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
//                    ADS1015_REG_CONFIG_DR_1600SPS; // 1600 samples per second (default)
                    ADS1015_REG_CONFIG_DR_1600SPS   | // 1600 samples per second (default)
                    ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)

  // Set PGA/voltage range
  config |= ADS1015_REG_CONFIG_PGA_2_048V;

  // Set single-ended input channel
  switch (ch)
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

  // Set 'start single-conversion' bit
  config |= ADS1015_REG_CONFIG_OS_SINGLE;

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
void OGR_SensorGas_ADS1015::update(void)
{
    // nothing to do - its all done in the timer()
}

void OGR_SensorGas_ADS1015::timer(void)
{
    if (get_reading()) {
        // update temp_valid state based on concentration measured
        update_status();
    } else {
        set_status(OGR_SensorGas::OGR_SensorGas_NoData);
    }
}
