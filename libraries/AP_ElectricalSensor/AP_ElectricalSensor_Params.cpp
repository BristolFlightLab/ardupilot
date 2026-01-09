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

#include "AP_ElectricalSensor_Params.h"
#include "AP_ElectricalSensor.h"

#if AP_ELECTRICAL_SENSOR_ENABLED

#ifndef AP_ELECTRICAL_SENSOR_SOURCE_ID_DEFAULT
#define AP_ELECTRICAL_SENSOR_SOURCE_ID_DEFAULT -1
#endif

const AP_Param::GroupInfo AP_ElectricalSensor_Params::var_info[] = {
    // @Param: TYPE
    // @DisplayName: Electrical Sensor Type
    // @Description: Enables electrical sensors
    // @Values: 0:Disabled, 1:DroneCAN
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO_FLAGS("TYPE", 1, AP_ElectricalSensor_Params, type, (float)Type::NONE, AP_PARAM_FLAG_ENABLE),

    // @Param: SRC
    // @DisplayName: Sensor Source
    // @Description: Sensor Source is used to designate which device's temperature report will be replaced by this temperature sensor's data. If 0 (None) then the data is only available via log. In the future a new Motor temperature report will be created for returning data directly.
    // @Values: 0: None, 1:DroneCAN-out on AP_Periph, 2:Servo
    // @User: Standard
    AP_GROUPINFO("SRC", 2, AP_ElectricalSensor_Params, source, (float)Source::None),

    // @Param: SRC_ID
    // @DisplayName: Sensor Source Identification
    // @Description: Sensor Source Identification is used to replace a specific instance of a system component's electrical report with the electrical sensor's. Examples: ELECx_SRC = 2 (ESC), ELECx_SRC_ID = 1 will set the voltage/current of servo 1
    AP_GROUPINFO("SRC_ID", 3, AP_ElectricalSensor_Params, source_id, AP_ELECTRICAL_SENSOR_SOURCE_ID_DEFAULT),

    AP_GROUPEND
};

AP_ElectricalSensor_Params::AP_ElectricalSensor_Params(void) {
    AP_Param::setup_object_defaults(this, var_info);
}

#endif // AP_ELECTRICAL_SENSOR_ENABLED
