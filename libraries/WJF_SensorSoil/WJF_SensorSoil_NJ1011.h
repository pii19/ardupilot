#pragma once

#include "WJF_SensorSoil.h"
#include "WJF_SensorSoil_Backend.h"

class WJF_SensorSoil_NJ1011 : public WJF_SensorSoil_Backend
{

public:
    // constructor
    WJF_SensorSoil_NJ1011(WJF_SensorSoil::WJF_SensorSoil_State &_state, AP_SerialManager &serial_manager);

    // static detection function
    static bool detect(AP_SerialManager &serial_manager);

    // update state
    void update(void);

private:
    // get a reading
    bool get_reading();
    void write_command();

    AP_HAL::UARTDriver *uart = nullptr;
    uint32_t last_reading_ms = 0;
    char linebuf[128];
    uint8_t linebuf_len = 0;
};
