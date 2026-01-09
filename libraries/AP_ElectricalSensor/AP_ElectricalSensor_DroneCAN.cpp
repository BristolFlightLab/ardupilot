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

#include "AP_ElectricalSensor_config.h"

#if AP_ELECTRICAL_SENSOR_DRONECAN_ENABLED

#include "AP_ElectricalSensor_DroneCAN.h"
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Math/AP_Math.h>

AP_ElectricalSensor_DroneCAN* AP_ElectricalSensor_DroneCAN::_drivers[];
uint8_t AP_ElectricalSensor_DroneCAN::_driver_instance;
HAL_Semaphore AP_ElectricalSensor_DroneCAN::_driver_sem;

extern const AP_HAL::HAL &hal;

const AP_Param::GroupInfo AP_ElectricalSensor_DroneCAN::var_info[] = {

    // @Param: MSG_ID
    // @DisplayName: Electrical sensor DroneCAN message ID
    // @Description: Sets the message circuit ID this backend listens for
    // @Range: 0 65535
    AP_GROUPINFO("MSG_ID", 1, AP_ElectricalSensor_DroneCAN, _ID, 0),

    // CHECK/UPDATE INDEX TABLE IN AP_ElectricalSensor_Backend.cpp WHEN CHANGING OR ADDING PARAMETERS

    AP_GROUPEND
};

AP_ElectricalSensor_DroneCAN::AP_ElectricalSensor_DroneCAN(AP_ElectricalSensor &front,
                                                             AP_ElectricalSensor::ElectricalSensor_State &state,
                                                             AP_ElectricalSensor_Params &params) :
    AP_ElectricalSensor_Backend(front, state, params)
{
    AP_Param::setup_object_defaults(this, var_info);
    _state.var_info = var_info;

    // Register self in static driver list
    WITH_SEMAPHORE(_driver_sem);
    _drivers[_driver_instance] = this;
    _driver_instance++;
}

// Subscript to incoming electrical messages
bool AP_ElectricalSensor_DroneCAN::subscribe_msgs(AP_DroneCAN* ap_dronecan)
{
    const auto driver_index = ap_dronecan->get_driver_index();

    return (Canard::allocate_sub_arg_callback(ap_dronecan, &handle_circuit_status, driver_index) != nullptr);
}

void AP_ElectricalSensor_DroneCAN::handle_circuit_status(AP_DroneCAN *ap_dronecan, const CanardRxTransfer& transfer, const uavcan_equipment_power_CircuitStatus &msg)
{
    WITH_SEMAPHORE(_driver_sem);

    for (uint8_t i = 0; i < _driver_instance; i++) {
        if ((_drivers[i] != nullptr) && (_drivers[i]->_ID.get() == msg.circuit_id)) {
            // Driver loaded and looking for this ID, set temp
            _drivers[i]->set_reading(msg.voltage, msg.current);
        }
    }
}

#endif // AP_ELECTRICAL_SENSOR_DRONECAN_ENABLED

