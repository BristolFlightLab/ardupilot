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

#if AP_ELECTRICAL_SENSOR_DRONECAN_ENABLED

#include "AP_ElectricalSensor_Backend.h"
#include <AP_Param/AP_Param.h>

#include <AP_DroneCAN/AP_DroneCAN.h>

class AP_ElectricalSensor_DroneCAN : public AP_ElectricalSensor_Backend {
public:
    AP_ElectricalSensor_DroneCAN(AP_ElectricalSensor &front, AP_ElectricalSensor::ElectricalSensor_State &state, AP_ElectricalSensor_Params &params);

    static bool subscribe_msgs(AP_DroneCAN* ap_dronecan);

    // Don't do anything in update, but still need to override the pure virtual method.
    void update(void) override {};

    static const struct AP_Param::GroupInfo var_info[];

private:

    static void handle_circuit_status(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_power_CircuitStatus &msg);

    // Static list of drivers
    static AP_ElectricalSensor_DroneCAN *_drivers[AP_ELECTRICAL_SENSOR_MAX_INSTANCES];
    static uint8_t _driver_instance;
    static HAL_Semaphore _driver_sem;

    // DroneCAN electrical ID to listen for
    AP_Int32 _ID;

};

#endif // AP_ELECTRICAL_SENSOR_DRONECAN_ENABLED
