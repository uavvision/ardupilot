--[[
  set_wind_test.lua: autotest helper for ahrs:set_wind (Plane.SetWindEKF3, local uavnav setWind patch).

  Each time SCR_USER1 changes to a new value above 0, calls ahrs:set_wind(SCR_USER2, SCR_USER3, SCR_USER4)
  once (north and east wind in m/s, the direction the air moves TO, and the 1-sigma uncertainty in m/s) and
  reports "SETWIND <n> accepted", "SETWIND <n> refused" or, for arguments outside the binding's ranges (wind
  -100..100, sigma 0.5..20, a Lua error that would stop a script calling it directly), "SETWIND <n> error";
  n is SCR_USER1.
--]]

local MAV_SEVERITY_INFO = 6
local REQUEST = Parameter("SCR_USER1")
local NORTH = Parameter("SCR_USER2")
local EAST = Parameter("SCR_USER3")
local SIGMA = Parameter("SCR_USER4")

local last = REQUEST:get()

local function update()
    local n = REQUEST:get()
    if n > 0 and n ~= last then
        last = n
        local called, ok = pcall(ahrs.set_wind, ahrs, NORTH:get(), EAST:get(), SIGMA:get())
        local result = (not called and "error") or (ok and "accepted") or "refused"
        gcs:send_text(MAV_SEVERITY_INFO, string.format("SETWIND %d %s", math.floor(n), result))
    end
    return update, 100
end

gcs:send_text(MAV_SEVERITY_INFO, "SETWIND ready")
return update()
