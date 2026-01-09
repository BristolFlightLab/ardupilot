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

#include "AP_ElectricalSensor_config.h"

#if AP_ELECTRICAL_SENSOR_ENABLED

#include "AP_ElectricalSensor_Params.h"

// declare backend class
class AP_ElectricalSensor_Backend;

class AP_ElectricalSensor
{
    friend class AP_ElectricalSensor_Backend;
    friend class AP_ElectricalSensor_DroneCAN;

public:

    // Constructor
    AP_ElectricalSensor();

    CLASS_NO_COPY(AP_ElectricalSensor);

    static AP_ElectricalSensor *get_singleton() { return _singleton; }

    // Return the number of electrical sensor instances
    uint8_t num_instances(void) const { return _num_instances; }

    // detect and initialise any available electrical sensors
    __INITFUNC__ void init();

    // Update the voltage / current for all electrical sensors
    void update();

    // return voltage from sensor
    bool get_voltage(float &voltage, const uint8_t instance = AP_ELECTRICAL_SENSOR_PRIMARY_INSTANCE) const;

    // return current from sensor
    bool get_current(float &current, const uint8_t instance = AP_ELECTRICAL_SENSOR_PRIMARY_INSTANCE) const;

    bool healthy(const uint8_t instance = AP_ELECTRICAL_SENSOR_PRIMARY_INSTANCE) const;

    // accessors to params
    AP_ElectricalSensor_Params::Type get_type(const uint8_t instance = AP_ELECTRICAL_SENSOR_PRIMARY_INSTANCE) const;
    AP_ElectricalSensor_Params::Source get_source(const uint8_t instance = AP_ELECTRICAL_SENSOR_PRIMARY_INSTANCE) const;
    int32_t get_source_id(const uint8_t instance = AP_ELECTRICAL_SENSOR_PRIMARY_INSTANCE) const;

    static const struct AP_Param::GroupInfo var_info[];
    static const struct AP_Param::GroupInfo *backend_var_info[AP_ELECTRICAL_SENSOR_MAX_INSTANCES];

protected:
    // parameters
    AP_ElectricalSensor_Params _params[AP_ELECTRICAL_SENSOR_MAX_INSTANCES];

private:
    static AP_ElectricalSensor *_singleton;

    // The ElectricalSensor_State structure is filled in by the backend driver
    struct ElectricalSensor_State {
        uint32_t    last_time_ms;          // time when the sensor was last read in milliseconds
        float       voltage;               // voltage (V)
        float       current;               // current (A)
        uint8_t     instance;              // instance number
        const struct AP_Param::GroupInfo *var_info;
    };

    ElectricalSensor_State _state[AP_ELECTRICAL_SENSOR_MAX_INSTANCES];
    AP_ElectricalSensor_Backend *drivers[AP_ELECTRICAL_SENSOR_MAX_INSTANCES];

    uint8_t     _num_instances;         // number of electrical sensors

#if HAL_LOGGING_ENABLED
    enum class LoggingType : uint8_t {
        All = 1,
        SourceNone = 2,
    };
    AP_Enum<LoggingType> _logging_type;
#endif

};

namespace AP {
    AP_ElectricalSensor &electrical_sensor();
};

#endif // AP_ELECTRICAL_SENSOR_ENABLED
