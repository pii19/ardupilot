#pragma once

#include "OGR_SensorTemp.h"
#include "OGR_SensorTemp_Backend.h"
#include <AP_HAL/I2CDevice.h>

class OGR_SensorTemp_ADT7410 : public OGR_SensorTemp_Backend
{

public:
    // static detection function
    static OGR_SensorTemp_Backend *detect(OGR_SensorTemp::OGR_SensorTemp_State &_state,
                                          AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    // update state
    void update(void);

private:
    // constructor
    OGR_SensorTemp_ADT7410(OGR_SensorTemp::OGR_SensorTemp_State &_state, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    void init();
    void timer();

    // get a reading
    bool get_reading(float &temperature);
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
};
