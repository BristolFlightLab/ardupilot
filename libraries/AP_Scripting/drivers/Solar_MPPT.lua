--[[
device driver for MPPT battery source
Based on BattMon_ANX.lua
--]]

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local PARAM_TABLE_KEY = 45
local PARAM_TABLE_PREFIX = "SOL_MPPT"

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- Setup EFI Parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 15), 'could not add param table')

--[[
  // @Param: SOL_MPPT_ENABLE
  // @DisplayName: Enable MPPT solar input
  // @Description: Enable MPPT solar input battery monitor support
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local SOL_MPPT_ENABLE = bind_add_param('ENABLE', 1, 0)

--[[
  // @Param: SOL_MPPT_CANDRV
  // @DisplayName: Set MPPT Solar CAN driver
  // @Description: Set MPPT Solar CAN driver
  // @Values: 0:None,1:1stCANDriver,2:2ndCanDriver
  // @User: Standard
--]]
local SOL_MPPT_CANDRV = bind_add_param('CANDRV', 2, 1)

--[[
  // @Param: SOL_MPPT_INDEX
  // @DisplayName: MPPT solar input battery index
  // @Description: MPPT solar input battery index
  // @Range: 1 10
  // @User: Standard
--]]
local SOL_MPPT_INDEX     = bind_add_param('INDEX',     3, 1)

-- Register for the CAN drivers
local driver

local CAN_BUF_LEN = 5
if SOL_MPPT_CANDRV:get() == 1 then
    driver = CAN.get_device(CAN_BUF_LEN)
elseif SOL_MPPT_CANDRV:get() == 2 then
    driver = CAN.get_device2(CAN_BUF_LEN)
end

if not driver then
    gcs:send_text(0, string.format("SOL_MPPT: Failed to load driver"))
    return
end

local assembly = {}
assembly.num_frames = 0
assembly.frames = {}

-- Only accept mppt.Stream msg on second driver
-- mppt.Stream is message ID 20009
-- Message ID is 16 bits left shifted by 8 in the CAN frame ID.
driver:add_filter(uint32_t(0xFFFF) << 8, uint32_t(20009) << 8)

function update_battery_monitor(voltage, current)
    local state = BattMonitorScript_State()
    state:voltage(voltage)
    state:current(current)
    battery:handle_scripting(SOL_MPPT:get()-1, state)
end

-- Interpret byte pair as float16
function interpret_float16(lsb, msb)
    local sign = 
    local exponent = 
    local mantissa = 
    
end

--[[
RX	17:10:10.102622	NFD	184E297C	F4 E2 02 19 00 00 00 91
RX	17:10:10.103547	NFD	184E297C	00 00 00 88 4B 00 00 31
RX	17:10:10.104511	NFD	184E297C	00 00 51
--]]

function process_packet()
    -- fault_flags
    -- assembly.frames[0].data(2)

    -- temperature
    -- assembly.frames[0].data(3)

    -- input_voltage
    local input_voltage = interpret_float16(
        assembly.frames[0].data(4),
        assembly.frames[0].data(5)
    )

    -- input_current
    local input_current = interpret_float16(
        assembly.frames[0].data(6),
        assembly.frames[1].data(0)
    )

    -- -- input_power
    -- assembly.frames[1].data(1)
    -- assembly.frames[1].data(2)

    -- -- output_voltage
    -- assembly.frames[1].data(3)
    -- assembly.frames[1].data(4)

    -- -- output_current
    -- assembly.frames[1].data(5)
    -- assembly.frames[1].data(6)

    -- -- output_power
    -- assembly.frames[2].data(0)
    -- assembly.frames[2].data(1)

    update_battery_monitor(input_voltage, input_current)
end

--[[
    read from CAN bus, updating battery backend
--]]
function read_can()
    while true do
        local frame = driver:read_frame()
        if not frame then
            return
        end
        
        -- Length of payload (bytes)
        frame:dlc()
        frame:data(0)
        frame:data(1)
        frame:data(2)
        frame:data(3)
        frame:data(4)
        frame:data(5)
        frame:data(6)
        -- tail byte
        frame:data(7)
        
        update_battery_monitor(10)
    end
end

function update()
    read_can()
    return update, 10
end

gcs:send_text(MAV_SEVERITY.INFO, "SOL_MPPT: Started")

return update()
