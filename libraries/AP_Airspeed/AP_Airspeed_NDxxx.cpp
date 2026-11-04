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
/*
  driver for ND series pressure sensors
  https://superiorsensors.com/pressure-sensors/industrial
 */

#include "AP_Airspeed_NDxxx.h"

#if AP_AIRSPEED_NDXXX_ENABLED

#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

#define NDXXX_I2C_ADDR0 0x28
#define NDXXX_I2C_ADDR1 0x29
#define NDXXX_I2C_ADDR2 0x2A
#define NDXXX_I2C_ADDR3 0x2B
#define NDXXX_I2C_ADDR4 0x2C
#define NDXXX_I2C_ADDR5 0x2D
#define NDXXX_I2C_ADDR6 0x2E
#define NDXXX_I2C_ADDR7 0x2F
#define NDXXX_I2C_ADDR8 0x30
#define NDXXX_I2C_ADDR9 0x31

bool AP_Airspeed_NDxxx::probe(uint8_t address)
{
    _dev = hal.i2c_mgr->get_device_ptr(get_bus(), address);
    if (!_dev) {
        return false;
    }

    WITH_SEMAPHORE(_dev->get_semaphore());

    _dev->set_retries(5);
    _model = get_model();
    return _model != Model::Unknown;
}

Model AP_Airspeed_NDxxx::get_model()
{
    uint8_t result[16];
    _dev->read(result, 16);
    const char* model = reinterpret_cast<char*>(&result[8]);

    if (strcmp(model, "ND210")) {
        return Model::ND210;
    }
    if (strcmp(model, "ND110")) {
        return Model::ND110;
    }
    if (strcmp(model, "ND120")) {
        return Model::ND120;
    }
    if (strcmp(model, "ND130")) {
        return Model::ND130;
    }
    if (strcmp(model, "ND005D")) {
        return Model::ND005D;
    }
    return Model::Unknown;
}

// probe and initialise the sensor
bool AP_Airspeed_NDxxx::init()
{
    const uint8_t addresses[] = {
        NDXXX_I2C_ADDR0,
        NDXXX_I2C_ADDR1,
        NDXXX_I2C_ADDR2,
        NDXXX_I2C_ADDR3,
        NDXXX_I2C_ADDR4,
        NDXXX_I2C_ADDR5,
        NDXXX_I2C_ADDR6,
        NDXXX_I2C_ADDR7,
        NDXXX_I2C_ADDR8,
        NDXXX_I2C_ADDR9,
    };
    bool found = false;
    for (uint8_t i=0; i<ARRAY_SIZE(addresses); i++) {
        found = probe(addresses[i]);

        if (found) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NDxxx AIRSPEED[%u]: Found bus %u addr 0x%02x", get_instance(), _dev->bus_num(), _dev->get_bus_address());
            break;
        }
    }

    if (!found) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NDxxx AIRSPEED[%u]: no sensor found on bus %u", get_instance(), get_bus());
        return false;
    }

    WITH_SEMAPHORE(_dev->get_semaphore());
    // Configure device range and rates
    setup();

    _dev->set_device_type(uint8_t(DevType::NDXXX));
    set_bus_id(_dev->get_bus_id());

    // drop to 2 retries for runtime
    _dev->set_retries(2);

    // Read at 50 Hz
    _dev->register_periodic_callback(20000,
                                     FUNCTOR_BIND_MEMBER(&AP_Airspeed_NDxxx::timer, void));
    return true;
}

enum class BandwidthLimit : uint8_t {
    HZ_1 = 0,
    HZ_2 = 1,
    HZ_5 = 2,
    HZ_10 = 3,
    HZ_20 = 4,
    HZ_50 = 5,
    HZ_100 = 6,
    HZ_200 = 7
};

static uint8_t get_mode_register_value(bool enable_notch_filter, BandwidthLimit bw_limit, bool enable_watchdog, RangeSelect range);
static uint8_t get_mode_register_value(bool enable_notch_filter, BandwidthLimit bw_limit, bool enable_watchdog, RangeSelect range) {
    uint8_t value = 0;
    if (enable_notch_filter) {
        value |= 0x80;
    }
    value |= (static_cast<uint8_t>(bw_limit) << 4);
    if (enable_watchdog) {
        value |= 0x08;
    }
    value |= static_cast<uint8_t>(range);
    return value;
}

// Ranges in inH2O
const float ranges_ND210[8] = { 0.25, 0.25, 0.5, 1.0, 2.0, 4.0, 5.0, 10.0 };
const float ranges_ND110[8] = { 0.5, 0.5, 0.5, 1.0, 2.0, 4.0, 5.0, 10.0 };
const float ranges_ND120[8] = { 1.0, 1.0, 2.0, 4.0, 5.0, 10.0, 15.0, 20.0 };
const float ranges_ND130[8] = { 2.0, 2.0, 2.0, 4.0, 5.0, 10.0, 20.0, 30.0 };
// NB: ranges for ND005D are in PSI, multiplied below
const float ranges_ND005D[8] = { 0.5, 0.5, 0.5, 0.8, 1.0, 2.0, 4.0, 5.0 };

#define PSI_TO_INCH_OF_H2O 27.7076f

static float get_range_inH2O(Model model, RangeSelect range);
static float get_range_inH2O(Model model, RangeSelect range) {
    switch (model) {
        case Model::ND210:
            return ranges_ND210[static_cast<uint8_t>(range)];
        case Model::ND110:
             return ranges_ND110[static_cast<uint8_t>(range)];
        case Model::ND120:
             return ranges_ND120[static_cast<uint8_t>(range)];
        case Model::ND130:
             return ranges_ND130[static_cast<uint8_t>(range)];
        case Model::ND005D:
             return PSI_TO_INCH_OF_H2O * ranges_ND005D[static_cast<uint8_t>(range)];
        default:
            return 0.0;
    }
}

// initialise the sensor
void AP_Airspeed_NDxxx::setup()
{
    WITH_SEMAPHORE(_dev->get_semaphore());
    // Setup rate and range
    // Currently use default values from datasheet
    RangeSelect selected_range = RangeSelect::Range6;
    uint8_t mode_val = get_mode_register_value(true, BandwidthLimit::HZ_200, false, selected_range);
    _dev->transfer(&mode_val, 1, nullptr, 0);
    _range_inH2O = get_range_inH2O(_model, selected_range);
}

// 50Hz timer
void AP_Airspeed_NDxxx::timer()
{
    uint8_t raw_bytes[4];
    if (!_dev->read((uint8_t *)&raw_bytes, sizeof(raw_bytes))) {
        return;
    }

    int16_t pressure_raw = (raw_bytes[0] << 8) | raw_bytes[1];
    float press_h2o = pressure_raw / (0.9f * INT16_MAX) * _range_inH2O;

    int16_t temp_raw = (raw_bytes[2] << 8) | raw_bytes[3];
    float temp = temp_raw / 256.0f;

    WITH_SEMAPHORE(sem);

    const uint32_t now = AP_HAL::millis();

    _press_sum += INCH_OF_H2O_TO_PASCAL * press_h2o;
    _temp_sum += temp;
    _press_count++;
    _temp_count++;
    _last_sample_time_ms = now;
}

// return the current differential_pressure in Pascal
bool AP_Airspeed_NDxxx::get_differential_pressure(float &pressure)
{
    WITH_SEMAPHORE(sem);

    if ((AP_HAL::millis() - _last_sample_time_ms) > 100) {
        return false;
    }

    if (_press_count > 0) {
        _pressure = _press_sum / _press_count;
        _press_count = 0;
        _press_sum = 0;
    }

    pressure = _pressure;
    return true;
}

// return the current temperature in degrees C, if available
bool AP_Airspeed_NDxxx::get_temperature(float &temperature)
{
    WITH_SEMAPHORE(sem);

    if ((AP_HAL::millis() - _last_sample_time_ms) > 100) {
        return false;
    }

    if (_temp_count > 0) {
        _temperature = _temp_sum / _temp_count;
        _temp_count = 0;
        _temp_sum = 0;
    }

    temperature = _temperature;
    return true;
}

#endif
