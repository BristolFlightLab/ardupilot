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

#include <AP_Param/AP_Param.h>

class AP_ElectricalSensor_Params {
public:
    static const struct AP_Param::GroupInfo var_info[];

    AP_ElectricalSensor_Params(void);

    CLASS_NO_COPY(AP_ElectricalSensor_Params);

    // temperature sensor types
    enum class Type : uint8_t {
        NONE                        = 0,
        DRONECAN                    = 1,
    };

    // option to map to another system component
    enum class Source : uint8_t {
        None                        = 0,
        DroneCAN                    = 1,
        Servo                       = 2,
    };

    AP_Enum<Type> type;             // 0=disabled, others see frontend enum TYPE

    AP_Enum<Source> source;         // library mapping
    AP_Int32 source_id;             // library instance mapping
};
