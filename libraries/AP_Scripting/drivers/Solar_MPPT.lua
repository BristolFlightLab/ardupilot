--[[
device driver for MPPT battery source
Based on BattMon_ANX.lua
--]]

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local PARAM_TABLE_KEY = 45
local PARAM_TABLE_PREFIX = "SOLMPPT_"

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- Setup EFI Parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 15), 'could not add param table')

--[[
  // @Param: SOLMPPT_ENABLE
  // @DisplayName: Enable MPPT solar input
  // @Description: Enable MPPT solar input battery monitor support
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local SOLMPPT_ENABLE = bind_add_param('ENABLE', 1, 0)

--[[
  // @Param: SOLMPPT_CANDRV
  // @DisplayName: Set MPPT Solar CAN driver
  // @Description: Set MPPT Solar CAN driver
  // @Values: 0:None,1:1stCANDriver,2:2ndCanDriver
  // @User: Standard
--]]
local SOLMPPT_CANDRV = bind_add_param('CANDRV', 2, 1)

--[[
  // @Param: SOLMPPT_INDEX
  // @DisplayName: MPPT solar input battery index
  // @Description: MPPT solar input battery index
  // @Range: 1 10
  // @User: Standard
--]]
local SOLMPPT_INDEX     = bind_add_param('INDEX', 3, 1)

-- Disable driver with parameter
if SOLMPPT_ENABLE:get() == 0 then
   gcs:send_text(0, string.format("SOLMPPT: disabled"))
   return
end

-- Register for the CAN drivers
local driver

local CAN_BUF_LEN = 20
if SOLMPPT_CANDRV:get() == 1 then
    driver = CAN.get_device(CAN_BUF_LEN)
elseif SOLMPPT_CANDRV:get() == 2 then
    driver = CAN.get_device2(CAN_BUF_LEN)
end

if not driver then
    gcs:send_text(0, string.format("SOLMPPT: Failed to load driver"))
    return
end

local assembly = {}
assembly.num_frames = 0
assembly.frames = {}

-- Only accept mppt.Stream msg on second driver
-- mppt.Stream is message ID 20009
-- Message ID is 16 bits left shifted by 8 in the CAN frame ID.
driver:add_filter(uint32_t(0xFFFF) << 8, uint32_t(20009) << 8)

function tail_byte_from_frame(frame)
    local data_length = frame:dlc()
    return frame:data(data_length-1)
end

function tail_byte_start_of_transfer(tail_byte)
    return tail_byte >> 7 & 0x01
end

function tail_byte_end_of_transfer(tail_byte)
    return tail_byte >> 6 & 0x01
end

function tail_byte_toggle(tail_byte)
    return tail_byte >> 5 & 0x01
end

function tail_byte_transfer_id(tail_byte)
    return tail_byte & 0x1f
end

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
    -- battery:handle_scripting(SOLMPPT:get()-1, state)
end

-- Interpret byte pair as float16
-- Adapted from: https://discuss.ardupilot.org/t/issues-with-interegrating-can-sensor-using-lua/115585/16
function interpret_float16(msb, lsb)
    local sign = (msb >> 7) == 0 and 1 or -1
    local exponent = (msb >> 2) & 0x1f
    local mantissa = ((msb & 0x03) << 8) | lsb

    if exponent == 0 and mantissa == 0 then
        return 0.0
    elseif exponent == 0 and mantissa ~= 0 then
        -- return sign * math.ldexp(mantissa / 0x400, -14)
    elseif exponent == 0x1f and mantissa == 0 then
        return sign * math.huge
    elseif exponent == 0x1f and mantissa ~=0 then
        return 0.0 / 0.0
    end
    
    exponent = exponent - 15
    local fraction = 1.0 + mantissa / 0x400
    return sign * fraction * (2 ^ exponent)
end

--[[
RX	17:10:10.102622	NFD	184E297C	F4 E2 02 19 00 00 00 91
RX	17:10:10.103547	NFD	184E297C	00 00 00 88 4B 00 00 31
RX	17:10:10.104511	NFD	184E297C	00 00 51
--]]

function process_packet()
    -- Check all transfer IDs match
    local tx_id_0 = tail_byte_transfer_id(tail_byte_from_frame(assembly.frames[0]))
    local tx_id_1 = tail_byte_transfer_id(tail_byte_from_frame(assembly.frames[1]))
    local tx_id_2 = tail_byte_transfer_id(tail_byte_from_frame(assembly.frames[2]))

    if tx_id_0 ~= tx_id_1 or tx_id_1 ~= tx_id_2 then
        -- Got frames from different transfers
        gcs:send_text(MAV_SEVERITY.INFO, "SOLMPPT: Multiple transfers")
        return
    end
    
    -- Check CRC?
    
    -- -- fault_flags
    -- assembly.frames[0]:data(2)

    -- -- temperature
    -- assembly.frames[0]:data(3)

    -- input_voltage
    local input_voltage = interpret_float16(
        assembly.frames[0]:data(5),
        assembly.frames[0]:data(4)
    )

    -- -- input_current
    local input_current = interpret_float16(
        assembly.frames[1]:data(0),
        assembly.frames[0]:data(6)
    )

    -- -- input_power
    -- assembly.frames[1]:data(2)
    -- assembly.frames[1]:data(1)

    -- output_voltage
    local output_voltage = interpret_float16(
        assembly.frames[1]:data(4),
        assembly.frames[1]:data(3)
    )

    -- output_current
    local output_current = interpret_float16(
        assembly.frames[1]:data(6),
        assembly.frames[1]:data(5)
    )

    -- -- output_power
    -- assembly.frames[2]:data(1)
    -- assembly.frames[2]:data(0)

    gcs:send_text(MAV_SEVERITY.INFO, string.format(
        "SOLMPPT: ic=%f, iv=%f, oc=%f, ov=%f", input_current, input_voltage, output_current, output_voltage
    ))
    
    update_battery_monitor(0, input_voltage, input_current)
    update_battery_monitor(1, output_voltage, output_current)
end

--[[
    read from CAN bus, updating battery backend
--]]
function read_can()
    while true do
        local frame = driver:read_frame()
        if not frame then
            gcs:send_text(MAV_SEVERITY.INFO, "SOLMPPT: No frame")
            return
        end
        if not frame:isExtended() then
            -- only want extended frames
            gcs:send_text(MAV_SEVERITY.INFO, "SOLMPPT: Not extended frame")
            return
        end

        local tail_byte = tail_byte_from_frame(frame)

        if assembly.num_frames == 0 then
            -- Check that tail_byte indicates start of transfer
            if not tail_byte_start_of_transfer(tail_byte) == 1 then
                -- Out of sync
                gcs:send_text(MAV_SEVERITY.INFO, "SOLMPPT: Sync: Not first frame")
                assembly.num_frames = 0
                return
            end
            assembly.frames[assembly.num_frames] = frame
            assembly.num_frames = assembly.num_frames + 1
            -- Minimal delay to avoid missing frames
            return 0
        end

        if assembly.num_frames == 1 then
            -- Check that tail_byte indicates mid tranfer
            if not tail_byte_toggle(tail_byte) == 1 then
                -- Out of sync
                gcs:send_text(MAV_SEVERITY.INFO, "SOLMPPT: Sync: Not middle frame")
                assembly.num_frames = 0
                return
            end
            assembly.frames[assembly.num_frames] = frame
            assembly.num_frames = assembly.num_frames + 1
            -- Minimal delay to avoid missing frames
            return 0
        end
        
        -- Reading final frame
        -- Check that tail_byte indicates end of transfer
        if not tail_byte_end_of_transfer(tail_byte) == 1 then
            -- Out of sync
            gcs:send_text(MAV_SEVERITY.INFO, "SOLMPPT: Sync: Not final frame")
            assembly.num_frames = 0
            return
        end
        
        assembly.frames[assembly.num_frames] = frame

        gcs:send_text(MAV_SEVERITY.INFO, "SOLMPPT: Processing packet")
        process_packet()
        assembly.num_frames = 0
        return
    end
end

function update()
    delay = read_can()
    if delay ~= 0 then
        return update, 5
    else
        return update, delay
    end
end

gcs:send_text(MAV_SEVERITY.INFO, "SOLMPPT: Started")

return update()
