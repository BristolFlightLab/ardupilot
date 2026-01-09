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
#include "AP_ElectricalSensor_Backend.h"

#include <AP_Logger/AP_Logger.h>
#include <AP_Servo_Telem/AP_Servo_Telem.h>

/*
  All backends use the same parameter table and set of indices. Therefore, two
  backends must not use the same index. The list of used indices and
  corresponding backends is below.

    1:   AP_ElectricalSensor_DroneCAN.cpp

  Usage does not need to be contiguous. The maximum possible index is 63.
*/

/*
    base class constructor.
    This incorporates initialisation as well.
*/
AP_ElectricalSensor_Backend::AP_ElectricalSensor_Backend(AP_ElectricalSensor &front,
                                                            AP_ElectricalSensor::ElectricalSensor_State &state,
                                                            AP_ElectricalSensor_Params &params):
    _front(front),
    _state(state),
    _params(params)
{
}

// returns true if a voltage / current has been recently updated
bool AP_ElectricalSensor_Backend::healthy(void) const
{
    return (_state.last_time_ms > 0) && (AP_HAL::millis() - _state.last_time_ms < 5000);
}

#if HAL_LOGGING_ENABLED
void AP_ElectricalSensor_Backend::Log_Write_ELEC() const
{
    // @LoggerMessage: ELEC
    // @Description: Electrical Sensor Data
    // @Field: TimeUS: Time since system startup
    // @Field: Instance: electrical sensor instance
    // @Field: Voltage: voltage
    // @Field: Current: current
    AP::logger().Write("ELEC",
            "TimeUS,"     "Instance,"       "Voltage,"   "Current" // labels
            "s"               "#"           "V"          "A"    ,  // units
            "F"               "-"           "0"          "0"    ,  // multipliers
            "Q"               "B"           "f"          "f"    ,  // types
     AP_HAL::micros64(), _state.instance, _state.voltage, _state.current);
}
#endif

void AP_ElectricalSensor_Backend::set_reading(const float voltage, const float current)
{
    {
        WITH_SEMAPHORE(_sem);
        _state.voltage = voltage;
        _state.current = current;
        _state.last_time_ms = AP_HAL::millis();
    }

    update_external_libraries(voltage, current);
}

void AP_ElectricalSensor_Backend::update_external_libraries(const float voltage, const float current)
{
#if AP_SERVO_TELEM_ENABLED
    AP_Servo_Telem *servo_telem;
    AP_Servo_Telem::TelemetryData servo_telem_data;
#endif

    switch ((AP_ElectricalSensor_Params::Source)_params.source.get()) {
        case AP_ElectricalSensor_Params::Source::DroneCAN:
            // Label only, used by AP_Periph
            break;

#if AP_SERVO_TELEM_ENABLED
        case AP_ElectricalSensor_Params::Source::Servo:
            servo_telem = AP_Servo_Telem::get_singleton();
            if (servo_telem == nullptr) {
                break;
            }
            servo_telem_data.voltage = voltage * 100;
            servo_telem_data.current = current * 100;
            servo_telem_data.present_types = AP_Servo_Telem::TelemetryData::Types::VOLTAGE | AP_Servo_Telem::TelemetryData::Types::CURRENT;
            servo_telem->update_telem_data(_params.source_id-1, servo_telem_data);
            break;
#endif // AP_SERVO_TELEM_ENABLED

        case AP_ElectricalSensor_Params::Source::None:
        default:
            break;
    }

}

#endif // AP_ELECTRICAL_SENSOR_ENABLED
