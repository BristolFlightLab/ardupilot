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

#include "AP_ElectricalSensor.h"

#if AP_ELECTRICAL_SENSOR_ENABLED

#include "AP_ElectricalSensor_DroneCAN.h"

#include <AP_Logger/AP_Logger.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_ElectricalSensor::var_info[] = {

    // SKIP INDEX 0

#if HAL_LOGGING_ENABLED
    // @Param: _LOG
    // @DisplayName: Logging
    // @Description: Enables electrical sensor logging
    // @Values: 0:Disabled, 1:Log all instances, 2: Log only instances with sensor source set to None
    // @User: Standard
    AP_GROUPINFO("_LOG", 1, AP_ElectricalSensor, _logging_type, 0),
#endif

    // SKIP Index 2-9 to be for parameters that apply to every sensor

    // @Group: 1_
    // @Path: AP_ElectricalSensor_Params.cpp
    AP_SUBGROUPINFO(_params[0], "1_", 10, AP_ElectricalSensor, AP_ElectricalSensor_Params),

    // @Group: 1_
    // @Path: AP_ElectricalSensor_DroneCAN.cpp
    AP_SUBGROUPVARPTR(drivers[0], "1_", 11, AP_ElectricalSensor, backend_var_info[0]),

#if AP_ELECTRICAL_SENSOR_MAX_INSTANCES >= 2
    // @Group: 2_
    // @Path: AP_ElectricalSensor_Params.cpp
    AP_SUBGROUPINFO(_params[1], "2_", 11, AP_ElectricalSensor, AP_ElectricalSensor_Params),

    // @Group: 2_
    // @Path: AP_ElectricalSensor_DroneCAN.cpp
    AP_SUBGROUPVARPTR(drivers[1], "2_", 20, AP_ElectricalSensor, backend_var_info[1]),
#endif

#if AP_ELECTRICAL_SENSOR_MAX_INSTANCES >= 3
    // @Group: 3_
    // @Path: AP_ElectricalSensor_Params.cpp
    AP_SUBGROUPINFO(_params[2], "3_", 12, AP_ElectricalSensor, AP_ElectricalSensor_Params),

    // @Group: 3_
    // @Path: AP_ElectricalSensor_DroneCAN.cpp
    AP_SUBGROUPVARPTR(drivers[2], "3_", 21, AP_ElectricalSensor, backend_var_info[2]),
#endif

#if AP_ELECTRICAL_SENSOR_MAX_INSTANCES >= 4
    // @Group: 4_
    // @Path: AP_ElectricalSensor_Params.cpp
    AP_SUBGROUPINFO(_params[3], "4_", 13, AP_ElectricalSensor, AP_ElectricalSensor_Params),

    // @Group: 4_
    // @Path: AP_ElectricalSensor_DroneCAN.cpp
    AP_SUBGROUPVARPTR(drivers[3], "4_", 22, AP_ElectricalSensor, backend_var_info[3]),
#endif

#if AP_ELECTRICAL_SENSOR_MAX_INSTANCES >= 5
    // @Group: 5_
    // @Path: AP_ElectricalSensor_Params.cpp
    AP_SUBGROUPINFO(_params[4], "5_", 14, AP_ElectricalSensor, AP_ElectricalSensor_Params),

    // @Group: 5_
    // @Path: AP_ElectricalSensor_DroneCAN.cpp
    AP_SUBGROUPVARPTR(drivers[4], "5_", 23, AP_ElectricalSensor, backend_var_info[4]),
#endif

#if AP_ELECTRICAL_SENSOR_MAX_INSTANCES >= 6
    // @Group: 6_
    // @Path: AP_ElectricalSensor_Params.cpp
    AP_SUBGROUPINFO(_params[5], "6_", 15, AP_ElectricalSensor, AP_ElectricalSensor_Params),

    // @Group: 6_
    // @Path: AP_ElectricalSensor_DroneCAN.cpp
    AP_SUBGROUPVARPTR(drivers[5], "6_", 24, AP_ElectricalSensor, backend_var_info[5]),
#endif

#if AP_ELECTRICAL_SENSOR_MAX_INSTANCES >= 7
    // @Group: 7_
    // @Path: AP_ElectricalSensor_Params.cpp
    AP_SUBGROUPINFO(_params[6], "7_", 16, AP_ElectricalSensor, AP_ElectricalSensor_Params),

    // @Group: 7_
    // @Path: AP_ElectricalSensor_DroneCAN.cpp
    AP_SUBGROUPVARPTR(drivers[6], "7_", 25, AP_ElectricalSensor, backend_var_info[6]),
#endif

#if AP_ELECTRICAL_SENSOR_MAX_INSTANCES >= 8
    // @Group: 8_
    // @Path: AP_ElectricalSensor_Params.cpp
    AP_SUBGROUPINFO(_params[7], "8_", 17, AP_ElectricalSensor, AP_ElectricalSensor_Params),

    // @Group: 8_
    // @Path: AP_ElectricalSensor_DroneCAN.cpp
    AP_SUBGROUPVARPTR(drivers[7], "8_", 26, AP_ElectricalSensor, backend_var_info[7]),
#endif

#if AP_ELECTRICAL_SENSOR_MAX_INSTANCES >= 9
    // @Group: 9_
    // @Path: AP_ElectricalSensor_Params.cpp
    AP_SUBGROUPINFO(_params[8], "9_", 18, AP_ElectricalSensor, AP_ElectricalSensor_Params),

    // @Group: 9_
    // @Path: AP_ElectricalSensor_DroneCAN.cpp
    AP_SUBGROUPVARPTR(drivers[8], "9_", 27, AP_ElectricalSensor, backend_var_info[8]),
#endif

    AP_GROUPEND
};

const AP_Param::GroupInfo *AP_ElectricalSensor::backend_var_info[AP_ELECTRICAL_SENSOR_MAX_INSTANCES];

// Default Constructor
AP_ElectricalSensor::AP_ElectricalSensor()
{
    AP_Param::setup_object_defaults(this, var_info);

    if (_singleton != nullptr) {
        AP_HAL::panic("AP_ElectricalSensor must be singleton");
    }
    _singleton = this;
}

// init - instantiate the electrical sensors
void AP_ElectricalSensor::init()
{
    // check init has not been called before
    if (_num_instances != 0) {
        return;
    }

    // create each instance
    for (uint8_t instance = 0; instance < AP_ELECTRICAL_SENSOR_MAX_INSTANCES; instance++) {
        _state[instance].instance = instance;

        switch (get_type(instance)) {
#if AP_ELECTRICAL_SENSOR_DRONECAN_ENABLED
            case AP_ElectricalSensor_Params::Type::DRONECAN:
                drivers[instance] = NEW_NOTHROW AP_ElectricalSensor_DroneCAN(*this, _state[instance], _params[instance]);
                break;
#endif
            case AP_ElectricalSensor_Params::Type::NONE:
            default:
                break;
        }

        // call init function for each backend
        if (drivers[instance] != nullptr) {
            if (_state[instance].var_info != nullptr) {
                // Load backend specific params
                backend_var_info[instance] = _state[instance].var_info;
                AP_Param::load_object_from_eeprom(drivers[instance], backend_var_info[instance]);
            }

            drivers[instance]->init();
            // _num_instances is actually the index for looping over instances
            // the user may have ELEC_TYPE=0 and ELEC2_TYPE=7, in which case
            // there will be a gap, but as we always check for drivers[instances] being nullptr
            // this is safe
            _num_instances = instance + 1;
        }
    }

    if (_num_instances > 0) {
        // param count could have changed
        AP_Param::invalidate_count();
    }
}

// update: - For all active instances update voltage / current and log ELEC
void AP_ElectricalSensor::update()
{
    for (uint8_t i=0; i<_num_instances; i++) {
        if (drivers[i] != nullptr && get_type(i) != AP_ElectricalSensor_Params::Type::NONE) {
            drivers[i]->update();

#if HAL_LOGGING_ENABLED
            const AP_Logger *logger = AP_Logger::get_singleton();
            const bool should_log = (_logging_type == LoggingType::All) ||
                                    ((_logging_type == LoggingType::SourceNone) && (_params[i].source == AP_ElectricalSensor_Params::Source::None));
            if (logger != nullptr && should_log) {
                drivers[i]->Log_Write_ELEC();
            }
#endif
        }
    }
}

AP_ElectricalSensor_Params::Type AP_ElectricalSensor::get_type(const uint8_t instance) const
{
    if (instance >= AP_ELECTRICAL_SENSOR_MAX_INSTANCES) {
        return AP_ElectricalSensor_Params::Type::NONE;
    }
    return (AP_ElectricalSensor_Params::Type)_params[instance].type.get();
}

// returns true if there is a voltage reading
bool AP_ElectricalSensor::get_voltage(float &voltage, const uint8_t instance) const
{
    if (!healthy(instance)) {
        return false;
    }

    voltage = _state[instance].voltage;
    return true;
}

// returns true if there is a current reading
bool AP_ElectricalSensor::get_current(float &current, const uint8_t instance) const
{
    if (!healthy(instance)) {
        return false;
    }

    current = _state[instance].current;
    return true;
}

bool AP_ElectricalSensor::healthy(const uint8_t instance) const
{
    return instance < _num_instances && drivers[instance] != nullptr && drivers[instance]->healthy();
}

AP_ElectricalSensor_Params::Source AP_ElectricalSensor::get_source(const uint8_t instance) const
{
    return healthy(instance) ? (AP_ElectricalSensor_Params::Source)_params[instance].source.get() : AP_ElectricalSensor_Params::Source::None;
}

int32_t AP_ElectricalSensor::get_source_id(const uint8_t instance) const
{
    return healthy(instance) ? _params[instance].source_id.get() : 0;
}

AP_ElectricalSensor *AP_ElectricalSensor::_singleton;

namespace AP {
AP_ElectricalSensor &electrical_sensor() {
    return *AP_ElectricalSensor::get_singleton();
}
};

#endif // AP_ELECTRICAL_SENSOR_ENABLED
