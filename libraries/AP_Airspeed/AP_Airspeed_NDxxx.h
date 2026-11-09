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
#pragma once

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_NDXXX_ENABLED

/*
  backend driver for airspeed from Superior ND series sensors over I2C
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <utility>

#include "AP_Airspeed_Backend.h"

enum class Model {
    ND210,
    ND110,
    ND120,
    ND130,
    ND005D,
    Unknown,
};

enum class RangeSelect : uint8_t {
    Range0 = 0,
    Range1 = 1,
    Range2 = 2,
    Range3 = 3,
    Range4 = 4,
    Range5 = 5,
    Range6 = 6,
    Range7 = 7
};

class AP_Airspeed_NDxxx : public AP_Airspeed_Backend
{
public:
    using AP_Airspeed_Backend::AP_Airspeed_Backend;

    ~AP_Airspeed_NDxxx(void) {
        delete _dev;
    }

    // probe and initialise the sensor
    bool init() override;

    // return the current differential_pressure in Pascal
    bool get_differential_pressure(float &pressure) override;

    // return the current temperature in degrees C, if available
    bool get_temperature(float &temperature) override;

private:
    void timer();

    Model get_model();
    char const* get_model_string() const;
    bool probe(uint8_t address);
    void setup();

    float _temp_sum;
    float _press_sum;
    uint32_t _temp_count;
    uint32_t _press_count;
    float _temperature;
    float _pressure;

    uint32_t _last_sample_time_ms;
    Model _model;
    float _range_inH2O;
    AP_HAL::I2CDevice *_dev;
};

#endif  // AP_AIRSPEED_NDXXX_ENABLED
