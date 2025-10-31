local function get_instance_power_W(instance)
    local v = battery:voltage(instance)
    local i = battery:current_amps(instance) or 0
    gcs:send_text(6, string.format("Instance %i: V=%f I=%f", instance, v, i))
    return v * i
end

local function get_battery_output_W(output_instance, input_instance)
    local output_W = get_instance_power_W(output_instance)
    local input_W = get_instance_power_W(input_instance)
    return output_W - input_W
end

local batt_consumed_J = 0.0

local last_read_time_ms = millis()

function update()
    -- Get the power for the configured batteries
    local now_ms = millis()
    local total_output_W = get_battery_output_W(2, 0)
    gcs:send_text(6, string.format("Total power: %f", total_output_W))

    -- Update the estimated energy consumed for each battery
    batt_consumed_J = batt_consumed_J + total_output_W * (now_ms - last_read_time_ms):tofloat() / 1000.0
    last_read_time_ms = now_ms

    -- Send a debug message with the values
    -- gcs:send_named_float("BAT1_J_CON", batt_consumed_J)
    gcs:send_text(6, string.format("BAT1_J_CON: %f", batt_consumed_J))

    return update, 1000 -- reschedule at 1Hz
end

return update()
