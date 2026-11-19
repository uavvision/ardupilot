--[[
  wind_on_gps_loss.lua: give EKF3 a wind when GPS is lost, and keep it while dead reckoning.

  Needs the local firmware patch ahrs:set_wind (branch Plane-4.5.4-uavnav-setwind): EKF3 accepts a wind only
  while it is dead reckoning, so the script just keeps offering it in steady flight; the first accepted call
  marks the start of dead reckoning.

  Which wind (SWND_SRC):
    0  always the wind set before flight (SWND_SPD, SWND_DIR: a forecast or a launch-site measurement)
    1  auto: if the EKF had real GPS in flight for SWND_GPS_S seconds, keep the wind it learned from GPS, unless
       a forecast is set and the learned wind is more than SWND_SRC_TOL from it; otherwise the forecast
  "Real GPS" = the primary receiver has a 3D fix, reports more than 5 m/s, and its position (within SWND_GPS_TOL)
  and speed (within 3 m/s) agree with the EKF's, i.e. the EKF is using it. A position typed in on the ground
  (speed 0) and a spoofed receiver the EKF rejects do not count. In SITL the EKF had learned 6.2 of 8 m/s after
  60 s of GPS in the air, so a longer SWND_GPS_S or a forecast to check against is worth having.

  When: in steady flight only (flying for SWND_DELAY s, not in TAKEOFF mode and not on an AUTO NAV_TAKEOFF,
  airspeed above SWND_AS_MIN). A wind given 0.3 s into a catapult launch locked a wrong wind in Replay of the
  svitlich flight (3.3 km at landing).

  Holding (SWND_HOLD = 1): once accepted, EK3_WIND_P_NSE is set to SWND_HOLD_NSE and EK3_WIND_PSCALE to 0 (in
  RAM, not saved), so the airspeed fusion does not walk the wind away; both are restored when real GPS is back
  for 5 s, on disarm or when disabled. A scripting restart during the hold is detected at start and the values
  are set to 0.1 / 1.0 (this aircraft's). Changing SWND_SPD or SWND_DIR while dead reckoning re-sends that wind
  (operator update; it overrides a learned wind).

  Replay of the svitlich flight (GPS jammed then spoofed, 25 min): landing error 8.2 km as flown, 1.9 km with
  the forecast wind given 7 s after launch and held.
--]]

local SCRIPT_VERSION = "1.0 19-11-26"
local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
local TAG = "[SWND] "
local MODE_TAKEOFF = 13
local MODE_AUTO = 10
local NAV_TAKEOFF = 22
local GPS_FIX_3D = 3
local GPS_MOVING_MPS = 5.0   -- a real fix in flight moves; a typed-in position does not
local GPS_BACK_MS = 5000     -- primary GPS good this long = GPS is back
local PERIOD_MS = 500

-- ---- parameters ------------------------------------------------------------------------------------------
local PARAM_TABLE_KEY = 171  -- keys used on the aircraft: 115, 139, 145, 147, 156, 159, 165, 191
local PARAM_TABLE_PREFIX = "SWND_"
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 12), "SWND: could not add param table")

local function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), "SWND: could not add " .. name)
    local p = Parameter()
    assert(p:init(PARAM_TABLE_PREFIX .. name), "SWND: could not find " .. name)
    return p
end

--[[
  // @Param: SWND_ENABLE
  // @DisplayName: Wind on GPS loss enable
  // @Values: 0:Disabled,1:Enabled
--]]
local SWND_ENABLE = bind_add_param("ENABLE", 1, 1)
--[[
  // @Param: SWND_SPD
  // @DisplayName: Wind speed set before flight
  // @Description: Forecast or measured wind speed at flight altitude; negative = none given
  // @Units: m/s
--]]
local SWND_SPD = bind_add_param("SPD", 2, -1)
--[[
  // @Param: SWND_DIR
  // @DisplayName: Wind direction set before flight
  // @Description: Direction the wind blows FROM, degrees true (as in forecasts)
  // @Units: deg
--]]
local SWND_DIR = bind_add_param("DIR", 3, 0)
--[[
  // @Param: SWND_SIGMA
  // @DisplayName: Wind uncertainty given to the EKF
  // @Range: 0.5 20
  // @Units: m/s
--]]
local SWND_SIGMA = bind_add_param("SIGMA", 4, 0.5)
--[[
  // @Param: SWND_SRC
  // @DisplayName: Wind source
  // @Values: 0:Set before flight,1:Auto (learned from GPS if available)
--]]
local SWND_SRC = bind_add_param("SRC", 5, 1)
--[[
  // @Param: SWND_GPS_S
  // @DisplayName: GPS time in flight to trust the learned wind
  // @Units: s
--]]
local SWND_GPS_S = bind_add_param("GPS_S", 6, 30)
--[[
  // @Param: SWND_DELAY
  // @DisplayName: Time flying before a wind is given
  // @Units: s
--]]
local SWND_DELAY = bind_add_param("DELAY", 7, 20)
--[[
  // @Param: SWND_AS_MIN
  // @DisplayName: Least airspeed to give a wind (0 = no check)
  // @Units: m/s
--]]
local SWND_AS_MIN = bind_add_param("AS_MIN", 8, 12)
--[[
  // @Param: SWND_HOLD
  // @DisplayName: Hold the wind while dead reckoning
  // @Values: 0:No,1:Yes
--]]
local SWND_HOLD = bind_add_param("HOLD", 9, 1)
--[[
  // @Param: SWND_HOLD_NSE
  // @DisplayName: EK3_WIND_P_NSE while holding
  // @Units: m/s/s
--]]
local SWND_HOLD_NSE = bind_add_param("HOLD_NSE", 10, 0.01)
--[[
  // @Param: SWND_GPS_TOL
  // @DisplayName: GPS agreement with the EKF
  // @Description: The primary GPS counts as in use only while its position is within this distance of the EKF's
  // @Units: m
--]]
local SWND_GPS_TOL = bind_add_param("GPS_TOL", 11, 100)
--[[
  // @Param: SWND_SRC_TOL
  // @DisplayName: Learned wind check against the forecast
  // @Description: With SWND_SRC 1 and a forecast set, the learned wind is used only if it is within this of the forecast; otherwise the forecast
  // @Units: m/s
--]]
local SWND_SRC_TOL = bind_add_param("SRC_TOL", 12, 3)

local EK3_WIND_P_NSE = Parameter("EK3_WIND_P_NSE")
local EK3_WIND_PSCALE = Parameter("EK3_WIND_PSCALE")

-- ---- firmware check ----------------------------------------------------------------------------------------
if not ahrs.set_wind then
    gcs:send_text(MAV_SEVERITY.ERROR, TAG .. "firmware has no ahrs:set_wind, script stopped")
    return
end
gcs:send_text(MAV_SEVERITY.INFO, TAG .. "Wind on GPS loss, version " .. SCRIPT_VERSION)

-- A scripting restart while the wind was held leaves the hold values in RAM; the script would then save them as
-- the "normal" values. Put this aircraft's values back (EK3_WIND_P_NSE 0.1, EK3_WIND_PSCALE 1.0, the defaults).
local NORMAL_NSE, NORMAL_PSCALE = 0.1, 1.0
if EK3_WIND_PSCALE:get() == 0 and math.abs(EK3_WIND_P_NSE:get() - SWND_HOLD_NSE:get()) < 1e-6 then
    EK3_WIND_P_NSE:set(NORMAL_NSE)
    EK3_WIND_PSCALE:set(NORMAL_PSCALE)
    gcs:send_text(MAV_SEVERITY.WARNING, TAG .. "wind hold left from a restart, EK3_WIND params set to 0.1 / 1.0")
end

-- ---- state -------------------------------------------------------------------------------------------------
local STATE = {IDLE=0, TRY=1, HELD=2}
local state = STATE.IDLE
local gps_flying_ms = 0      -- time flying with a real primary GPS
local gps_good_since = nil   -- while holding: when the primary GPS became good
local last_ms = nil
local saved_nse, saved_pscale = nil, nil
local sent_spd, sent_dir = nil, nil  -- the set-before-flight values last sent
local warned_far = false             -- learned-vs-forecast warning sent this flight

local function now_ms()
    return millis():toint()
end

-- The primary GPS is good AND the EKF is using it: 3D fix, moving, and its position and speed agree with the
-- EKF's. A spoofed receiver that the EKF rejects (e.g. 3D fix, 56 m/s, another continent) never agrees, so it
-- neither ends the hold nor counts as GPS time for the learned wind.
local function primary_gps_good()
    local i = gps:primary_sensor()
    if gps:status(i) < GPS_FIX_3D or gps:ground_speed(i) <= GPS_MOVING_MPS then
        return false
    end
    local ekf_loc = ahrs:get_location()
    if not ekf_loc or gps:location(i):get_distance(ekf_loc) > SWND_GPS_TOL:get() then
        return false
    end
    return math.abs(gps:ground_speed(i) - ahrs:groundspeed_vector():length()) < 3.0
end

local function log_event(ev, wn, we, sigma, src)
    -- Ev: 1 accepted, 2 re-sent, 3 GPS back (restored), 4 disarmed (restored), 5 disabled (restored)
    logger:write("SWND", "Ev,WN,WE,Sig,Src", "BfffB", ev, wn, we, sigma, src)
end

local function hold_on()
    if SWND_HOLD:get() < 0.5 or saved_nse then
        return
    end
    saved_nse, saved_pscale = EK3_WIND_P_NSE:get(), EK3_WIND_PSCALE:get()
    EK3_WIND_P_NSE:set(SWND_HOLD_NSE:get())
    EK3_WIND_PSCALE:set(0)
end

local function hold_off()
    if saved_nse then
        EK3_WIND_P_NSE:set(saved_nse)
        EK3_WIND_PSCALE:set(saved_pscale)
        saved_nse, saved_pscale = nil, nil
    end
end

-- The wind to give now: north, east (m/s, the direction the air moves TO), source (0 set, 1 learned), or nil.
-- force_set: use the set-before-flight wind even when a learned one is available (operator update).
local function wind_to_give(force_set)
    local spd = SWND_SPD:get()
    local fn, fe = nil, nil
    if spd >= 0 then
        local dir = math.rad(SWND_DIR:get())
        fn, fe = -spd * math.cos(dir), -spd * math.sin(dir)
    end
    if not force_set and SWND_SRC:get() >= 0.5 and gps_flying_ms >= SWND_GPS_S:get() * 1000 then
        local w = ahrs:wind_estimate()
        -- a learned wind far from the forecast (short or poor GPS phase) is not trusted
        if not fn or math.sqrt((w:x() - fn)^2 + (w:y() - fe)^2) <= SWND_SRC_TOL:get() then
            return w:x(), w:y(), 1
        end
        if not warned_far then
            warned_far = true
            gcs:send_text(MAV_SEVERITY.WARNING, string.format(
                "%slearned wind N %.1f E %.1f far from forecast, using forecast", TAG, w:x(), w:y()))
        end
    end
    if not fn then
        return nil
    end
    return fn, fe, 0
end

local function steady_flight()
    local mode = vehicle:get_mode()
    if not vehicle:get_likely_flying() or mode == MODE_TAKEOFF then
        return false
    end
    if mode == MODE_AUTO and mission:get_current_nav_id() == NAV_TAKEOFF then
        return false  -- a catapult launch flown in AUTO
    end
    local flying_ms = vehicle:get_time_flying_ms()
    if type(flying_ms) ~= "number" then
        flying_ms = flying_ms:toint()  -- uint32_t comes as userdata
    end
    if flying_ms < SWND_DELAY:get() * 1000 then
        return false
    end
    local as_min = SWND_AS_MIN:get()
    if as_min > 0 then
        local as = ahrs:airspeed_estimate()
        if not as or as < as_min then
            return false
        end
    end
    return true
end

local function give(ev)
    local wn, we, src = wind_to_give(ev == 2)
    if not wn then
        return false
    end
    local sigma = math.max(0.5, math.min(20, SWND_SIGMA:get()))
    if not ahrs:set_wind(wn, we, sigma) then
        return false  -- not dead reckoning (GPS or other aiding in use): try again later
    end
    sent_spd, sent_dir = SWND_SPD:get(), SWND_DIR:get()
    log_event(ev, wn, we, sigma, src)
    gcs:send_text(MAV_SEVERITY.NOTICE, string.format("%swind %s N %.1f E %.1f m/s given (%s)", TAG,
        ev == 1 and "set" or "re-sent", wn, we, src == 1 and "learned from GPS" or "set before flight"))
    return true
end

local function reset(ev)
    if saved_nse then
        hold_off()
        log_event(ev, 0, 0, 0, 0)
        local why = ({[3] = "GPS back", [4] = "disarmed", [5] = "disabled"})[ev] or "reset"
        gcs:send_text(MAV_SEVERITY.INFO, TAG .. why .. ", wind params restored")
    end
    gps_good_since = nil
end

-- ---- loop --------------------------------------------------------------------------------------------------
local function update()
    local t = now_ms()
    local dt = last_ms and (t - last_ms) or 0
    last_ms = t

    if SWND_ENABLE:get() < 0.5 then
        reset(5)
        state = STATE.IDLE
        return update, PERIOD_MS
    end

    if not arming:is_armed() then
        if state ~= STATE.IDLE then
            reset(4)
        end
        state, gps_flying_ms, sent_spd, sent_dir, warned_far = STATE.IDLE, 0, nil, nil, false
        return update, PERIOD_MS
    end

    if vehicle:get_likely_flying() and primary_gps_good() then
        gps_flying_ms = gps_flying_ms + dt
    end

    if state == STATE.IDLE then
        state = STATE.TRY
    end

    if state == STATE.TRY then
        if steady_flight() and give(1) then
            hold_on()
            state = STATE.HELD
        end
    elseif state == STATE.HELD then
        if primary_gps_good() then
            gps_good_since = gps_good_since or t
            if t - gps_good_since >= GPS_BACK_MS then
                reset(3)
                state = STATE.TRY  -- ready for the next outage
            end
        else
            gps_good_since = nil
            -- the operator changed SWND_SPD or SWND_DIR while dead reckoning: send that wind (it overrides a
            -- learned one). sent_spd/sent_dir are the values when the wind was last given.
            if SWND_SPD:get() >= 0 and (SWND_SPD:get() ~= sent_spd or SWND_DIR:get() ~= sent_dir) then
                if not give(2) then
                    sent_spd, sent_dir = SWND_SPD:get(), SWND_DIR:get()  -- refused: do not retry every tick
                end
            end
        end
    end
    return update, PERIOD_MS
end

return update()
