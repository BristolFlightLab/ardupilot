--[[
device driver for MPPT battery source
Based on BattMon_ANX.lua
--]]

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local PARAM_TABLE_KEY = 55
local PARAM_TABLE_PREFIX = "SOLMPPT2_"

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- Setup EFI Parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 15), 'could not add param table')

--[[
  // @Param: SOLMPPT2_ENABLE
  // @DisplayName: Enable MPPT solar input
  // @Description: Enable MPPT solar input battery monitor support
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local SOLMPPT2_ENABLE = bind_add_param('ENABLE', 1, 0)

--[[
  // @Param: SOLMPPT2_DRVIDX
  // @DisplayName: MPPT DroneCAN driver index
  // @Description: MPPT DroneCAN driver index
  // @Values: 0:first driver,1:second driver
  // @User: Standard
--]]
local SOLMPPT2_DRVIDX = bind_add_param('DRVIDX', 2, 0)

--[[
  // @Param: SOLMPPT2_INDEX
  // @DisplayName: MPPT solar input battery index
  // @Description: MPPT solar input battery index. Associated BATTx_MONITOR needs to be 29 (Scripting)
  // @Range: 1 10
  // @User: Standard
--]]
local SOLMPPT2_INDEX     = bind_add_param('INDEX', 3, 1)

-- Disable driver with parameter
if SOLMPPT2_ENABLE:get() == 0 then
   gcs:send_text(0, string.format("SOLMPPT2: disabled"))
   return
end

local MPPTSTREAM_ID = 20009
local MPPTSTREAM_SIGNATURE = uint64_t(0xDD7096B2, 0x55FB6358)

-- Setup subscription to mppt.Stream message
local mppt_stream_handle = DroneCAN_Handle(
    SOLMPPT2_DRVIDX:get(), MPPTSTREAM_SIGNATURE, MPPTSTREAM_ID
)
mppt_stream_handle:subscribe()

function update_battery_monitor(index, voltage, current)
    local state = BattMonitorScript_State()
    state:voltage(voltage)
    state:healthy(true)
    state:cell_count(0)
    state:capacity_remaining_pct(255)
    state:cycle_count(65535)
    state:current_amps(current)
    state:consumed_mah(0.0 / 0.0)
    state:consumed_wh(0.0 / 0.0)
    state:temperature(0.0 / 0.0)
    battery:handle_scripting(index, state)
end

--[[
    unpack a float16 into a floating point number
--]]
local function unpackFloat16(v16)
    -- Extract the sign (bit 15), exponent (bits 10–14) and fraction (bits 0–9)
    local sign     = (v16 >> 15) & 0x1
    local exponent = (v16 >> 10) & 0x1F
    local fraction = v16 & 0x3FF

    local value
    if exponent == 0 then
        if fraction == 0 then
            -- Zero (positive or negative)
            value = 0.0
        else
            -- Subnormal numbers (exponent = -14, no implicit leading 1)
            value = (fraction / 1024.0) * 2.0^-14
        end
    elseif exponent == 0x1F then
        if fraction == 0 then
            -- Infinity (positive or negative)
            value = math.huge
        else
            -- NaN (Not a Number)
            value = 0/0
        end
    else
        -- Normalized numbers: implicit 1 before the fraction and exponent bias of 15.
        value = (1 + fraction / 1024.0) * 2.0^(exponent - 15)
    end

    -- Apply the sign bit
    if sign == 1 then
        value = -value
    end

    return value
end

--[[
RX	17:10:10.102622	NFD	184E297C	F4 E2 02 19 00 00 00 91
RX	17:10:10.103547	NFD	184E297C	00 00 00 88 4B 00 00 31
RX	17:10:10.104511	NFD	184E297C	00 00 51
--]]

function check_mppt_stream()
    local payload, nodeid = mppt_stream_handle:check_message()
    if not payload then
        return
    end

    local fault_flags, temperature, input_voltage_raw, input_current_raw,
        input_power_raw, output_voltage_raw, output_current_raw,
        output_power_raw = string.unpack("BbHHHHHH", payload)

    -- input_voltage
    local input_voltage = unpackFloat16(input_voltage_raw)

    -- -- input_current
    local input_current = unpackFloat16(input_current_raw)

    -- input_power
    -- local input_power = unpackFloat16(input_power_raw)

    -- output_voltage
    local output_voltage = unpackFloat16(output_voltage_raw)

    -- output_current
    local output_current = unpackFloat16(output_current_raw)

    -- output_power
    -- local output_power = unpackFloat16(output_power_raw)

    gcs:send_text(MAV_SEVERITY.INFO, string.format(
        "SOLMPPT2: ic=%f, iv=%f, oc=%f, ov=%f", input_current, input_voltage, output_current, output_voltage
    ))
    
    update_battery_monitor(SOLMPPT2_INDEX:get()-1, input_voltage, input_current)
end

local last_low_rate_ms = uint32_t(0)

local function update()
    local now = millis()
    if now - last_low_rate_ms >= 1000 then
        last_low_rate_ms = now
        check_mppt_stream()
    end

    return update, 10
end

gcs:send_text(MAV_SEVERITY.INFO, "SOLMPPT2: Started")

return update, 1000
