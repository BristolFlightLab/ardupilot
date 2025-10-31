local function get_instance_power_W(instance)
    local v = battery:voltage(instance)
    local i = battery:current_amps(instance) or 0
    return v * i
end

local function get_battery_output_W(output_instance, input_instance)
    local output_W = get_instance_power_W(output_instance)
    local input_W = get_instance_power_W(input_instance)
    return output_W - input_W
end

local last_read_time_ms = millis()

local function compute_consumption_J(output_power_W, time_delta_ms)
    return output_power_W * time_delta_ms:tofloat() / 1000.0
end

local batt1_consumed_J = 0.0
local batt2_consumed_J = 0.0

function update()
    -- Get the power for the configured batteries
    local now_ms = millis()
    local total_output_1_W = get_battery_output_W(2, 0)
    local total_output_2_W = get_battery_output_W(2, 0)
    gcs:send_text(6, string.format("Total power 1: %f", total_output_1_W))
    gcs:send_text(6, string.format("Total power 2: %f", total_output_2_W))

    -- Update the estimated energy consumed for each battery
    local time_delta_ms = now_ms - last_read_time_ms
    batt1_consumed_J = batt1_consumed_J + compute_consumption_J(total_output_1_W, time_delta_ms)
    batt2_consumed_J = batt2_consumed_J + compute_consumption_J(total_output_2_W, time_delta_ms)
    last_read_time_ms = now_ms

    -- Send a debug message with the values
    -- gcs:send_named_float("BAT1_J_CON", batt1_consumed_J)
    -- gcs:send_named_float("BAT2_J_CON", batt2_consumed_J)
    gcs:send_text(6, string.format("BAT1_J_CON: %f", batt1_consumed_J))
    gcs:send_text(6, string.format("BAT2_J_CON: %f", batt2_consumed_J))

    return update, 1000 -- reschedule at 1Hz
end

return update()
