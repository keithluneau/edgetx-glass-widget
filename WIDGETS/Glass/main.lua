--[[
  Glass - "glass cockpit" telemetry widget for EdgeTX color radios
  ArduPilot over CRSF / ELRS / Crossfire (native CRSF sensors)

  Adapts to the screen: fonts, margins and tape widths scale from
  320x240 up to 800x480 (anything in between, any aspect ratio).
  Drawing cost (Lua instruction count) does NOT grow with resolution, so it
  stays under EdgeTX's per-refresh CPU limit on large screens too.

  Install:  copy this file to  SD:/WIDGETS/Glass/main.lua
  Setup:    Model > Telemetry > "Discover new sensors" (with ArduPilot powered
            and linked), then add "Glass" to a main-view layout with a single full-size
            zone. It does not need the widget's own full-screen mode.

  Sensors used (CRSF names): Ptch Roll Hdg Yaw Alt GAlt VSpd GSpd ASpd GPS Sats
                             FM RxBt Curr Capa RQly 1RSS
  Needs a recent EdgeTX (2.9+) for the flight-mode text (FM) sensor.
  Assumes the radio reports telemetry in metric units (km/h, metres); the Imperial
  option only changes what is displayed.

  Heading: shown as the GPS course over ground (Hdg), i.e. where the aircraft is
  actually going. It is held at low groundspeed during a flight, and before moving
  it falls back to the AHRS yaw (passthrough yaw, then the Yaw sensor).

  Fast attitude: ArduPilot only sends the standard CRSF attitude frame at ~1 Hz.
  If ArduPilot's CRSF passthrough is enabled (RC_OPTIONS +256, the same setting
  Yaapu uses), this widget reads attitude, altitude, vertical speed, speed and AHRS
  yaw from the passthrough frames (several times per second, depending on the link)
  and falls back to the standard sensors when they are absent. Attitude, altitude,
  vertical speed, speed and heading are lightly smoothed (fixed).
  Don't keep the Yaapu widget active in the same model while using this one:
  both read the same frame queue.
]]

local options = {
  { "Imperial", BOOL, 0 },        -- 1 = mph / ft / ft per s
  { "WarnVolts", STRING, "0" },     -- low-voltage warning, total pack volts, e.g. 12.2 (0 = off)
  { "WarnLQ",   VALUE, 0, 0, 100 },  -- link-quality warning at or below this % (0 = off)
  { "WarnRSSI", VALUE, 0, 0, 130 },  -- RSSI warning: enter 95 to warn at -95 dBm or weaker (0 = off)
  { "WarnMAh",  VALUE, 0, 0, 30000 }  -- mAh warning: flashes at or above this many mAh used (0 = off)
}

-- cached globals (fewer Lua instructions per call)
local floor, ceil, abs, sin, cos, rad = math.floor, math.ceil, math.abs, math.sin, math.cos, math.rad
local fmt = string.format
local exp = math.exp
local drawLine, drawText, fillR = lcd.drawLine, lcd.drawText, lcd.drawFilledRectangle
local CC = CUSTOM_COLOR
local XS = XSSIZE or TINSIZE or SMLSIZE   -- smallest font this firmware offers
local band, rshift = bit32.band, bit32.rshift
local popFn = crossfireTelemetryPop

local C = {}   -- colours
local L = {}   -- layout (all derived from the screen size)

local CARD = { [0] = "N", [90] = "E", [180] = "S", [270] = "W" }
local ATT_TC = 0.06   -- attitude smoothing time constant (s)
local HDG_TC = 0.30   -- heading smoothing time constant (s)
local GPS_TRACK_MIN_KMH = 10 -- GPS course is unreliable at very low groundspeeds
local GPS_TRACK_HYST = 2     -- km/h of hysteresis around that threshold

---------------------------------------------------------------------------
-- helpers (EdgeTX Lua wants integers for drawing calls)
---------------------------------------------------------------------------
local function I(v) return floor(v + 0.5) end

local function clamp(v, lo, hi)
  if v < lo then return lo elseif v > hi then return hi end
  return v
end

local function col(c) lcd.setColor(CC, c) end

local function fillRect(x, y, w, h, c)
  if w <= 0 or h <= 0 then return end
  lcd.setColor(CC, c)
  fillR(I(x), I(y), I(w), I(h), CC)
end

local function box(x, y, w, h, c)
  lcd.setColor(CC, c)
  lcd.drawRectangle(I(x), I(y), I(w), I(h), CC)
end

-- 2px thick line
local function line(x1, y1, x2, y2, c, thick)
  lcd.setColor(CC, c)
  x1, y1, x2, y2 = I(x1), I(y1), I(x2), I(y2)
  drawLine(x1, y1, x2, y2, SOLID, CC)
  if thick then
    if abs(x2 - x1) >= abs(y2 - y1) then
      drawLine(x1, y1 + 1, x2, y2 + 1, SOLID, CC)
    else
      drawLine(x1 + 1, y1, x2 + 1, y2, SOLID, CC)
    end
  end
end

local function text(x, y, s, c, flags)
  lcd.setColor(CC, c)
  drawText(I(x), I(y), s, (flags or 0) + CC)
end

local function dot(x, y, r, c)
  lcd.setColor(CC, c)
  lcd.drawFilledCircle(I(x), I(y), I(r), CC)
end

local function tv(name)
  local v = getValue(name)
  if type(v) == "number" then return v end
  return 0
end

local atan2 = math.atan2 or math.atan

local function initColors()
  C.sky     = lcd.RGB(25, 115, 195)
  C.gnd     = lcd.RGB(115, 78, 36)
  C.white   = lcd.RGB(255, 255, 255)
  C.black   = lcd.RGB(0, 0, 0)
  C.panel   = lcd.RGB(14, 17, 22)
  C.green   = lcd.RGB(0, 230, 90)
  C.yellow  = lcd.RGB(255, 210, 0)
  C.red     = lcd.RGB(255, 50, 40)
  C.magenta = lcd.RGB(255, 0, 255)
  C.grey    = lcd.RGB(130, 140, 150)
end

-- Everything is derived from the actual screen size.
--   fs / fm : small / medium font flags      sh / mh : their approx. pixel heights
--   k       : scale factor relative to a 480x272 screen
-- Static geometry (bank scale, aircraft symbol) is computed once here so the
-- per-frame code stays cheap.
-- real text size on this radio (falls back to the estimate if sizeText is missing)
local function measure(s, flags, dw, dh)
  if lcd.sizeText then
    local ok, w, h = pcall(lcd.sizeText, s, flags)
    if ok and type(w) == "number" and type(h) == "number" and w > 0 and h > 0 then
      return w, h
    end
  end
  return dw, dh
end

local function computeLayout()
  local W, H = LCD_W, LCD_H
  local k = H / 272
  L.W, L.H, L.k = W, H, k

  -- fonts: {small font, medium font, est. small height, est. medium height}
  -- (480x272 and 800x480 use the same fonts; only very small screens go one size down)
  local tiers = {
    { XS,      SMLSIZE, 10, 14 },
    { SMLSIZE, MIDSIZE, 13, 20 },
  }
  local t = tiers[(H < 260) and 1 or 2]
  L.fs, L.fm = t[1], t[2]

  -- measure the real glyph sizes so spacing fits whatever font this radio uses
  local _, hs = measure("Ag", L.fs, 0, t[3])
  local _, hm = measure("Ag", L.fm, 0, t[4])
  L.sh = clamp(I(hs), 8, 60)
  L.mh = clamp(I(hm), 10, 80)
  L.w2   = (measure("00", L.fs, L.sh * 1.2, 0))
  L.w3   = (measure("000", L.fs, L.sh * 1.8, 0))
  L.wTel = (measure("NO TELEMETRY", L.fm, L.mh * 7, 0))
  L.wHome = (measure("HOME", L.fs, L.sh * 2.6, 0))

  L.tickMaj = math.max(6, I(10 * k))
  L.tickMin = math.max(3, I(5 * k))
  L.hthick  = math.max(2, I(2 * k))

  L.topH  = math.max(I(H * 0.09), L.sh + 8)
  L.botH  = math.max(I(H * 0.21), L.mh + 2 * L.sh + 12)
  L.hdgH  = math.max(I(L.botH * 0.5), L.tickMaj + L.sh + 4)
  if L.hdgH + L.sh + 8 > L.botH then L.botH = L.hdgH + L.sh + 8 end
  L.tapeW = math.max(I(W * 0.15), 52)
  L.pfdX  = L.tapeW
  L.pfdW  = W - 2 * L.tapeW
  L.pfdY  = L.topH
  L.pfdH  = H - L.topH - L.botH
  L.cx    = L.pfdX + L.pfdW / 2
  L.cy    = L.pfdY + L.pfdH / 2
  L.botY  = L.pfdY + L.pfdH
  -- tape titles: use the largest font in which the longest title still fits the tape
  L.tFont, L.showUnits = XS, false
  local tfonts = { L.fs, SMLSIZE, XS }
  for i = 1, #tfonts do
    if (measure("IAS km/h", tfonts[i], L.sh * 5, 0)) <= L.tapeW - 8 then
      L.tFont, L.showUnits = tfonts[i], true
      break
    end
  end

  -- tick labels on the tapes and the heading tape: one font size down at the
  -- large text sizes, so more of them fit (the green value boxes keep the big font)
  L.lblFont = (L.fs == MIDSIZE or L.fs == DBLSIZE) and SMLSIZE or L.fs
  local _, lh = measure("Ag", L.lblFont, 0, L.sh)
  L.lblH = clamp(I(lh), 8, 60)
  L.lw3 = (measure("000", L.lblFont, L.lblH * 1.8, 0))

  -- heading tape: label every 10 degrees if the labels fit, else every 20 or 30
  local sp10 = L.pfdW / 9                      -- pixels per 10 degrees
  L.hLblStep = 30
  if L.lw3 + 8 <= sp10 then L.hLblStep = 10
  elseif L.lw3 + 8 <= 2 * sp10 then L.hLblStep = 20 end

  -- battery voltage: largest font in which "00.0V" fits the left block
  local vfonts = { L.fm, MIDSIZE, SMLSIZE, XS }
  L.voltFont = XS
  for i = 1, #vfonts do
    if (measure("00.0V", vfonts[i], L.mh * 3, 0)) <= L.tapeW - 8 then
      L.voltFont = vfonts[i]
      break
    end
  end
  local _, vh = measure("Ag", L.voltFont, 0, L.mh)
  L.voltH = clamp(I(vh), 8, 80)

  -- battery A / mAh lines: shrink the font if "00000mAh" is wider than the left block
  L.batFont = L.fs
  if (measure("00000mAh", L.fs, L.sh * 5, 0)) > L.tapeW - 8 then
    L.batFont = SMLSIZE
    if (measure("00000mAh", SMLSIZE, L.sh * 5, 0)) > L.tapeW - 8 then L.batFont = XS end
  end
  local _, bhh = measure("Ag", L.batFont, 0, L.sh)
  L.batH = clamp(I(bhh), 8, 60)

  -- "NO HOME" placeholder: one line if it fits, else wrap to two lines,
  -- and if even "HOME" is too wide use the smallest font as well
  local maxW = math.max(40, L.tapeW - 8)
  L.nhFont, L.nhLines = L.fs, 1
  if (measure("NO HOME", L.fs, L.sh * 4.2, 0)) > maxW then
    L.nhLines = 2
    if L.wHome > maxW then L.nhFont = XS end
  end
  local _, nhh = measure("Ag", L.nhFont, 0, L.sh)
  L.nhH = clamp(I(nhh), 8, 60)

  -- flight timer (centre of the row under the heading tape): medium font if the row is tall enough
  local rowH = L.botH - L.hdgH - 6
  L.timerFont = (rowH >= L.mh) and L.fm or L.fs
  L.timerH = (rowH >= L.mh) and L.mh or L.sh

  -- post-flight summary box: medium font if five rows fit inside the attitude area
  L.sumFont, L.sumH = L.fm, L.mh
  if L.sh + 7 * (L.mh + 4) + 16 > L.pfdH - 8 then L.sumFont, L.sumH = L.fs, L.sh end
  L.sumW = math.min(L.pfdW - 12, (measure("MAX DIST 99.9 mi", L.sumFont, L.sumH * 9, 0)) + 24)

  L.ppd = L.pfdH / 50                    -- pixels per degree of pitch
  local s = L.pfdW / 336
  L.hmaj, L.hmin = 40 * s, 20 * s        -- ladder half-widths

  -- static bank scale: flat list of x1,y1,x2,y2 (integers)
  local R = L.pfdH * 0.40
  local marks = { -60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60 }
  local b = {}
  for i = 1, #marks do
    local a = marks[i]
    local ar = rad(a)
    local len = (a % 30 == 0) and L.tickMaj or L.tickMin
    b[#b + 1] = I(L.cx + R * sin(ar))
    b[#b + 1] = I(L.cy - R * cos(ar))
    b[#b + 1] = I(L.cx + (R - len) * sin(ar))
    b[#b + 1] = I(L.cy - (R - len) * cos(ar))
  end
  L.bank = b
  L.markR = R - L.tickMaj - 2
  L.markDot = math.max(3, I(4 * k))

  -- static aircraft symbol
  local wx, inner = 70 * s, 24 * s
  L.ac = {
    ax1 = I(L.cx - wx), ax2 = I(L.cx - inner),
    bx1 = I(L.cx + inner), bx2 = I(L.cx + wx),
    y = I(L.cy), yd = I(L.cy) + math.max(5, I(9 * k)),
    r = math.max(2, I(3 * k))
  }
end

---------------------------------------------------------------------------
-- telemetry
---------------------------------------------------------------------------
-- ArduPilot passthrough packets carried in custom CRSF frames (0x80 / 0x7F)
--   0x5006: roll (bits 0-10, 0.2 deg, +900 offset), pitch (bits 11-20, 0.2 deg, +450 offset)
--   0x5005: vertical speed (bits 0-8), horizontal speed (bits 9-16), yaw (bits 17-27, 0.2 deg)
--           the horizontal speed is ArduPilot's airspeed estimate when it has one,
--           otherwise groundspeed (values > 9.9 m/s carry 1 m/s resolution)
--   0x5004: altitude above home (bits 12-24, decimetres, 10^n scaled)
local function ptPacket(w, id, val, now)
  local d = w.d
  if id == 0x5006 then
    d.ptRoll  = (band(val, 0x7FF) - 900) * 0.2
    d.ptPitch = (band(rshift(val, 11), 0x3FF) - 450) * 0.2
    d.ptTime = now

  elseif id == 0x5005 then
    -- Yaw
    d.ptYaw = band(rshift(val, 17), 0x7FF) * 0.2
    d.ptYawTime = now

    -- Vertical speed: bit 0 = x10 flag, bits 1-7 = dm/s, bit 8 = sign
    local vs = band(rshift(val, 1), 0x7F) * ((band(val, 1) == 1) and 10 or 1) * 0.1
    if band(rshift(val, 8), 1) == 1 then vs = -vs end
    d.ptVs, d.ptVsTime = vs, now

    -- Horizontal speed: bit 9 = x10 flag, bits 10-16 = dm/s
    d.ptSpd = band(rshift(val, 10), 0x7F) * ((band(rshift(val, 9), 1) == 1) and 10 or 1) * 0.36
    d.ptSpdTime = now

  elseif id == 0x5004 then
    -- Altitude above home: bits 12-13 = power of 10, bits 14-23 = value (dm), bit 24 = sign
    local a = band(rshift(val, 14), 0x3FF) * (10 ^ band(rshift(val, 12), 3)) * 0.1
    if band(rshift(val, 24), 1) == 1 then a = -a end
    d.ptAlt, d.ptAltTime = a, now
  end
end

local function readPassthrough(w, now)
  if not popFn then return end
  for _ = 1, 12 do                       -- bounded work per call
    local cmd, data = popFn()
    if cmd == nil then return end
    if (cmd == 0x80 or cmd == 0x7F) and data and #data >= 7 then
      local t = data[1]
      if t == 0xF0 then                  -- single packet
        ptPacket(w, data[3] * 256 + data[2],
                 data[7] * 16777216 + data[6] * 65536 + data[5] * 256 + data[4], now)
      elseif t == 0xF1 then              -- array of packets, 6 bytes each
        local base = 3
        while base + 5 <= #data do
          ptPacket(w, data[base + 1] * 256 + data[base],
                   data[base + 5] * 16777216 + data[base + 4] * 65536
                   + data[base + 3] * 256 + data[base + 2], now)
          base = base + 6
        end
      end
    end
  end
end

-- Arming is detected from the flight-mode text, which ArduPilot sends slowly, so by the
-- time we see it the aircraft may already have moved. While on the ground we keep a short
-- history of GPS positions (one per 0.5 s) and take the newest one that is at least
-- 2.5 s old, which is where the aircraft was when it was actually armed.
local function pickPrearm(w, now)
  local bestT, bi, oldT, oi
  for i = 1, 8 do
    local t = w.hTime[i]
    if t then
      if (now - t) >= 250 and (bestT == nil or t > bestT) then bestT, bi = t, i end
      if oldT == nil or t < oldT then oldT, oi = t, i end
    end
  end
  local i = bi or oi
  if i then return w.hLat[i], w.hLon[i] end
  return nil, nil
end

local function poll(w)
  local d = w.d
  local k = 180 / math.pi
  local now = getTime()                  -- 10 ms ticks

  readPassthrough(w, now)

  -- standard CRSF attitude sensors (ArduPilot sends these at ~1 Hz) as fallback
  local npitch, nroll = tv("Ptch"), tv("Roll")
  npitch = npitch * k                  -- EdgeTX reports CRSF attitude in radians
  nroll  = nroll * k
  local ptFresh = d.ptTime and (now - d.ptTime) < 100
  if ptFresh then
    d.pitch, d.roll = d.ptPitch, d.ptRoll
  else
    d.pitch, d.roll = npitch, nroll
  end

  -- optional smoothing: eases the display toward each new attitude sample so the
  -- horizon moves continuously instead of in steps (adds a small lag)
  local dta = (now - (w.aT or now)) / 100
  w.aT = now
  if w.pS == nil then
    w.pS, w.rS = d.pitch, d.roll
  elseif dta > 0 then
    if dta > 0.5 then dta = 0.5 end
    local a = 1 - exp(-dta / ATT_TC)
    w.pS = w.pS + (d.pitch - w.pS) * a
    local dr = d.roll - w.rS
    if dr > 180 then dr = dr - 360 elseif dr < -180 then dr = dr + 360 end
    w.rS = w.rS + dr * a
    if w.rS > 180 then w.rS = w.rS - 360 elseif w.rS < -180 then w.rS = w.rS + 360 end
  end
  d.pitch, d.roll = w.pS, w.rS

  -- Heading is the GPS course over ground (native CRSF "Hdg" sensor), which points where
  -- the aircraft is actually moving (handy for navigating to home). It is only trustworthy
  -- with a fix and some groundspeed, so:
  --   moving (>= 10 km/h, 2 km/h hysteresis):  use the GPS track
  --   slower, during a flight:                 hold the last good track
  --   otherwise (on the ground, before moving): AHRS yaw (passthrough, then the Yaw sensor)
  d.sats = tv("Sats")
  d.gs   = tv("GSpd")                 -- km/h, native CRSF GPS telemetry
  d.as   = tv("ASpd")                 -- km/h

  local hdg
  local minKmh = w.trackOn and (GPS_TRACK_MIN_KMH - GPS_TRACK_HYST) or GPS_TRACK_MIN_KMH
  w.trackOn = d.sats >= 6 and d.gs >= minKmh
  if w.trackOn then
    -- Hdg may legitimately be 0 (north), so validity comes from the fix and groundspeed
    hdg = tv("Hdg") % 360
    w.lastGpsTrack = hdg
  elseif d.sats >= 6 and w.lastGpsTrack ~= nil then
    hdg = w.lastGpsTrack              -- course is noisy near a standstill: hold the last one
  elseif d.ptYawTime and (now - d.ptYawTime) < 200 then
    hdg = d.ptYaw
  else
    local yaw = tv("Yaw")
    hdg = (yaw ~= 0) and (yaw * k) or 0
  end

  -- Smooth heading using the shortest angular path, including across north
  -- (for example, 359 -> 1 degrees should move forward by 2 degrees).
  hdg = hdg % 360
  local hdt = (now - (w.hdgSmoothTime or now)) / 100
  w.hdgSmoothTime = now
  if w.hdgSmooth == nil then
    w.hdgSmooth = hdg
  elseif hdt > 0 then
    if hdt > 0.5 then hdt = 0.5 end
    local ha = 1 - exp(-hdt / HDG_TC)
    local dh = ((hdg - w.hdgSmooth + 540) % 360) - 180
    w.hdgSmooth = (w.hdgSmooth + dh * ha) % 360
  end
  d.hdg = w.hdgSmooth

  -- altitude / vertical speed / speed: prefer the passthrough values (same source
  -- as the OSD and Yaapu), fall back to the standard CRSF sensors
  local altRaw = tv("Alt")
  if altRaw == 0 then altRaw = tv("GAlt") end
  local vsRaw = tv("VSpd")            -- m/s
  -- (passthrough altitude reads 0 until ArduPilot has a position, so wait for GPS)
  if d.ptAltTime and (now - d.ptAltTime) < 300 and d.sats >= 6 then altRaw = d.ptAlt end
  if d.ptVsTime and (now - d.ptVsTime) < 300 then vsRaw = d.ptVs end
  if d.ptSpdTime and (now - d.ptSpdTime) < 300 then
    d.spd, d.spdTitle = d.ptSpd, "SPD"     -- airspeed estimate if ArduPilot has one, else GS
  elseif d.as > 0 then
    d.spd, d.spdTitle = d.as, "IAS"
  else
    d.spd, d.spdTitle = d.gs, "GS"
  end

  -- smoothing: low-pass the vertical speed, and dead-reckon the altitude with it
  -- between (slow) altitude updates so the tape moves continuously
  local dts = (now - (w.sT or now)) / 100
  w.sT = now
  if w.altS == nil or abs(altRaw - w.altS) > 60 then
    w.altS, w.vsS = altRaw, vsRaw
  elseif dts > 0 then
    if dts > 1 then dts = 1 end
    w.vsS = w.vsS + (vsRaw - w.vsS) * clamp(dts * 4, 0, 1)
    w.altS = w.altS + w.vsS * dts
    w.altS = w.altS + (altRaw - w.altS) * clamp(dts * 2.5, 0, 1)
  end
  d.alt, d.vs = w.altS, w.vsS

  -- speed: same kind of low-pass as the vertical speed (removes the 1-2 Hz stepping)
  if w.spdS == nil or abs(d.spd - w.spdS) > 40 then
    w.spdS = d.spd
  elseif dts > 0 then
    w.spdS = w.spdS + (d.spd - w.spdS) * clamp(dts * 3, 0, 1)
  end
  d.spd = w.spdS

  d.volt = tv("RxBt")
  d.curr = tv("Curr")
  d.capa = tv("Capa")
  d.lq   = tv("RQly")
  d.rssi = tv("1RSS")

  local fm = getValue("FM")
  if type(fm) == "string" and fm ~= "" then
    -- ArduPilot appends '*' to the mode name while disarmed
    d.armed = string.sub(fm, -1) ~= "*"
    d.mode = (string.gsub(fm, "%*", ""))
  else
    d.mode = "---"
    d.armed = false
  end

  d.lost = (d.lq == 0 and d.rssi == 0 and d.volt == 0)

  -- position / home
  d.hasPos = false
  local gps = getValue("GPS")
  if type(gps) == "table" and gps.lat and gps.lon and d.sats >= 6 then
    d.lat, d.lon = gps.lat, gps.lon
    d.hasPos = true
    -- position history while on the ground (see pickPrearm)
    if not d.armed and not w.flying and (now - (w.hT or -1000)) >= 50 then
      w.hT = now
      local i = ((w.hI or 0) % 8) + 1
      w.hI = i
      w.hLat[i], w.hLon[i], w.hTime[i] = gps.lat, gps.lon, now
    end
    -- home on the first good fix (so the arrow works before flight); it is re-set at arming below
    if not w.home and d.sats >= 8 then
      w.home = { lat = gps.lat, lon = gps.lon }
    end
  end
  -- Flight record (timer + maximums). Armed state comes from the flight-mode text; it is
  -- only used while that text is valid, and a disarm must last 2 s to end the flight, so a
  -- telemetry glitch can't reset the timer or end the flight early.
  if d.mode ~= "---" then
    if d.armed then
      w.disarmT = nil
      if not w.flying then                      -- new arming: start a fresh record
        w.flying = true
        w.armT0 = now
        w.lastGpsTrack = nil             -- heading follows the nose until we are moving
        w.maxAlt, w.maxSpd, w.maxDist = 0, 0, 0
        w.minV, w.maxCurr, w.capa0, w.capaLast = nil, 0, d.capa, d.capa
        d.showSummary = false
        if d.hasPos then                        -- home = position at the moment of arming
          local la, lo = pickPrearm(w, now)
          w.home = { lat = la or d.lat, lon = lo or d.lon }
        end
      end
    elseif w.flying then
      w.disarmT = w.disarmT or now
      if now - w.disarmT > 200 then             -- disarmed for 2 s: the flight is over
        w.flying = false
        w.lastGpsTrack = nil             -- on the ground again: back to the nose heading
        d.flightSecs = (w.disarmT - w.armT0) / 100
        if d.flightSecs >= 10 then              -- ignore short arming tests
          d.sumAlt, d.sumSpd, d.sumDist = w.maxAlt, w.maxSpd, w.maxDist
          d.sumMinV, d.sumMaxA = w.minV, w.maxCurr
          d.sumUsed = math.max(0, (w.capaLast or 0) - (w.capa0 or 0))
          d.showSummary = true
        end
      end
    end
  end
  if w.flying then d.flightSecs = (now - w.armT0) / 100 end


  d.dist, d.bearing = 0, nil
  if w.home and d.hasPos then
    local dlat = (d.lat - w.home.lat) * 111320
    local dlon = (d.lon - w.home.lon) * 111320 * cos(rad(w.home.lat))
    d.dist = math.sqrt(dlat * dlat + dlon * dlon)
    d.bearing = (math.deg(atan2(-dlon, -dlat)) + 360) % 360   -- bearing from aircraft to home
  end

  -- flight maximums (only while flying)
  if w.flying then
    if d.alt > w.maxAlt then w.maxAlt = d.alt end
    if d.spd > w.maxSpd then w.maxSpd = d.spd end
    if d.dist > w.maxDist then w.maxDist = d.dist end
    if d.volt > 0 and (w.minV == nil or d.volt < w.minV) then w.minV = d.volt end
    if d.curr > w.maxCurr then w.maxCurr = d.curr end
    w.capaLast = d.capa
  end
  d.flying = w.flying
end

---------------------------------------------------------------------------
-- drawing: attitude indicator
---------------------------------------------------------------------------
local stripX, stripY, stripN = {}, {}, {}   -- reused every frame (no allocation)

local function drawHorizon(d)
  local pitch = d.pitch
  local r = rad(d.roll)
  local sinr, cosr = sin(r), cos(r)
  local cr = cosr
  if abs(cr) < 0.02 then cr = (cr < 0) and -0.02 or 0.02 end
  local tanr = sinr / cr

  local ppd = L.ppd
  local off = pitch * ppd
  local hx = L.cx + sinr * off
  local hy = L.cy + cosr * off

  -- above and below the horizon each get their own colour, so every pixel of the
  -- attitude box is painted exactly once (less fill work per frame)
  local base, over = C.sky, C.gnd                  -- above the line / below the line
  if cosr < 0 then base, over = C.gnd, C.sky end   -- inverted

  -- strip count adapts to the roll angle (1 strip when level, max 64) so the
  -- instruction count is small and independent of screen resolution
  local n = ceil(L.pfdW * abs(tanr) / 3)   -- stair height <= 3 px (the horizon line covers it)
  if n < 1 then n = 1 elseif n > 64 then n = 64 end
  local step = L.pfdW / n
  local px0 = L.pfdX
  local y = hy - tanr * (px0 + step * 0.5 - hx)
  local dy = tanr * step
  local top = L.pfdY
  local bottom = L.pfdY + L.pfdH
  local xi = px0
  lcd.setColor(CC, base)
  for i = 1, n do
    local nx = (i == n) and (px0 + L.pfdW) or floor(px0 + step * i)
    local yy = y
    if yy < top then yy = top elseif yy > bottom then yy = bottom end
    local yi = floor(yy)
    stripX[i], stripY[i], stripN[i] = xi, yi, nx
    if yi > top then fillR(xi, top, nx - xi, yi - top, CC) end
    xi = nx
    y = y - dy
  end
  lcd.setColor(CC, over)
  for i = 1, n do
    local yi = stripY[i]
    if yi < bottom then fillR(stripX[i], yi, stripN[i] - stripX[i], bottom - yi, CC) end
  end

  -- horizon line. EdgeTX does not draw diagonal lines whose end points are far
  -- off screen, so the line is clipped to the attitude box by hand first.
  lcd.setColor(CC, C.white)
  local xmin, xmax = L.pfdX, L.pfdX + L.pfdW - 1
  local ymin, ymax = L.pfdY, L.pfdY + L.pfdH - 1
  local t0, t1 = -100000, 100000       -- visible range along the line
  if abs(cosr) > 0.0001 then
    local ta, tb = (xmin - hx) / cosr, (xmax - hx) / cosr
    if ta > tb then ta, tb = tb, ta end
    if ta > t0 then t0 = ta end
    if tb < t1 then t1 = tb end
  elseif hx < xmin or hx > xmax then
    t1 = t0 - 1
  end
  if abs(sinr) > 0.0001 then
    local ta, tb = (hy - ymax) / sinr, (hy - ymin) / sinr
    if ta > tb then ta, tb = tb, ta end
    if ta > t0 then t0 = ta end
    if tb < t1 then t1 = tb end
  elseif hy < ymin or hy > ymax then
    t1 = t0 - 1
  end
  if t0 < t1 then
    local lx1, ly1 = floor(hx + cosr * t0), floor(hy - sinr * t0)
    local lx2, ly2 = floor(hx + cosr * t1), floor(hy - sinr * t1)
    -- centred on the true horizon and thick enough to cover the stair-steps
    -- of the strip edge, so the sky/ground boundary looks clean
    local th = L.hthick
    local need = ceil(step * abs(tanr)) + 1
    if need > th then th = need end
    if th > 9 then th = 9 end
    local j0 = -floor(th / 2)
    local ddy = ly2 - ly1
    if ddy >= -1 and ddy <= 1 then
      -- (nearly) level: one filled rectangle instead of th separate lines
      local xa = (lx1 < lx2) and lx1 or lx2
      local ya = (ly1 < ly2) and ly1 or ly2
      fillR(xa, ya + j0, abs(lx2 - lx1) + 1, th + abs(ddy), CC)
    else
      for j = j0, j0 + th - 1 do
        drawLine(lx1, ly1 + j, lx2, ly2 + j, SOLID, CC)
      end
    end
  end

  -- pitch ladder: only the lines that can be visible (about 10)
  local half = L.pfdH * 0.5 - 4
  local p0 = floor((pitch - 26) / 5) * 5
  local sh, sh2, fs = L.sh, L.sh / 2, L.fs
  for p = p0, p0 + 55, 5 do
    if p ~= 0 then
      local o = (pitch - p) * ppd
      if o > -half and o < half then
        local major = (p % 10 == 0)
        local hl = major and L.hmaj or L.hmin
        local px, py = L.cx + sinr * o, L.cy + cosr * o
        local dx, dyy = cosr * hl, -sinr * hl
        drawLine(floor(px - dx), floor(py - dyy), floor(px + dx), floor(py + dyy), SOLID, CC)
        if major then
          local lbl = fmt("%d", abs(p))
          drawText(floor(px - dx - L.w2 - 4), floor(py - dyy - sh2), lbl, fs + CC)
          drawText(floor(px + dx + 4),        floor(py + dyy - sh2), lbl, fs + CC)
        end
      end
    end
  end

  -- bank scale (precomputed ticks) + moving bank marker
  local b = L.bank
  for i = 1, #b, 4 do
    drawLine(b[i], b[i + 1], b[i + 2], b[i + 3], SOLID, CC)
  end
  drawLine(b[21] + 1, b[22], b[23] + 1, b[24], SOLID, CC)   -- thicker 0 mark
  local mr = rad(clamp(d.roll, -90, 90))
  dot(L.cx + L.markR * sin(mr), L.cy - L.markR * cos(mr), L.markDot, C.yellow)

  -- fixed aircraft symbol
  local a = L.ac
  lcd.setColor(CC, C.yellow)
  fillR(a.ax1, a.y, a.ax2 - a.ax1 + 2, 2, CC)         -- left wing, 2 px thick
  fillR(a.ax2, a.y, 2, a.yd - a.y + 1, CC)            -- left tip
  fillR(a.bx1, a.y, a.bx2 - a.bx1 + 1, 2, CC)         -- right wing
  fillR(a.bx1, a.y, 2, a.yd - a.y + 1, CC)            -- right tip
  dot(L.cx, L.cy, a.r, C.yellow)
end

---------------------------------------------------------------------------
-- drawing: vertical tapes (speed left, altitude right)
---------------------------------------------------------------------------
local function vtape(x, w, val, half, tick, lab, ticksRight, title, valText, boxW, rightPad)
  local y, h = L.pfdY, L.pfdH
  local mid = y + h / 2
  local ppu = (h / 2) / half
  local boxH = L.mh + 6
  local yTop = y + L.sh + 5
  local yBot = y + h - 2
  local gapLo, gapHi = mid - boxH / 2 - 2, mid + boxH / 2 + 2
  local sh2, fs = L.lblH / 2, L.lblFont      -- tick-label font and half height
  local tMaj, tMin = L.tickMaj, L.tickMin

  fillRect(x, y, w, h, C.panel)
  lcd.setColor(CC, C.white)

  local v = floor((val - half) / tick) * tick
  local vmax = val + half
  while v <= vmax do
    local py = mid - (v - val) * ppu
    local major = (v % lab == 0)
    local mg = major and sh2 or 0        -- labelled ticks need room for the text too
    if py > yTop + mg and py < yBot - mg and (py < gapLo - mg or py > gapHi + mg) then
      local tl = major and tMaj or tMin
      local pyi = floor(py)
      if ticksRight then
        drawLine(x + w - tl, pyi, x + w - 1, pyi, SOLID, CC)
        if major then drawText(x + w - tl - 3, floor(py - sh2), fmt("%d", v), fs + RIGHT + CC) end
      else
        drawLine(x, pyi, x + tl, pyi, SOLID, CC)
        if major then drawText(x + tl + 3, floor(py - sh2), fmt("%d", v), fs + CC) end
      end
    end
    v = v + tick
  end

  -- both value boxes sit the same distance (pad) in from the screen edge; on the altitude
  -- side that keeps the box clear of the VSI bar, the speed side mirrors it
  local bx = ticksRight and (x + (rightPad or 0)) or (x + w - boxW - (rightPad or 0))
  fillRect(bx, mid - boxH / 2, boxW, boxH, C.black)
  box(bx, mid - boxH / 2, boxW, boxH, C.white)
  text(bx + boxW / 2, mid - L.mh / 2 - 1, valText, C.green, L.fm + CENTER)
  text(x + w / 2, y + 2, title, C.grey, L.tFont + CENTER)
  box(x, y, w, h, C.grey)
  return mid
end

local function drawTapes(d, imperial)
  -- speed: prefer airspeed, fall back to groundspeed
  local spd = d.spd or 0
  local title = d.spdTitle or "GS"
  local unitTxt = "km/h"
  if imperial then spd = spd * 0.6214; unitTxt = "mph" end
  if L.showUnits then title = title .. " " .. unitTxt end
  local sideGap = math.max(6, I(8 * L.k))
  local barW = math.max(4, I(5 * L.k))
  vtape(0, L.tapeW, spd, 30, 5, 10, true, title,
        fmt("%d", I(spd)), L.tapeW - sideGap - barW - 2, barW + 3)

  -- altitude
  local alt = d.alt
  local altUnit = "m"
  local half, tick, lab = 40, 10, 20
  if imperial then alt = alt * 3.281; altUnit = "ft"; half, tick, lab = 120, 20, 40 end
  local atitle = L.showUnits and ("ALT " .. altUnit) or "ALT"
  local ax = L.pfdX + L.pfdW
  local mid = vtape(ax, L.tapeW, alt, half, tick, lab, false, atitle,
                    fmt("%d", I(alt)), L.tapeW - sideGap - barW - 2, barW + 3)

  -- vertical speed bar (+/- 5 m/s full scale)
  local vsn = clamp(d.vs / 5, -1, 1)
  local bh = vsn * (L.pfdH / 2 - L.sh - 6)
  local c = (vsn >= 0) and C.green or C.yellow
  if bh >= 0 then fillRect(ax + L.tapeW - barW - 1, mid - bh, barW, bh, c)
  else fillRect(ax + L.tapeW - barW - 1, mid, barW, -bh, c) end
end

---------------------------------------------------------------------------
-- drawing: heading tape
---------------------------------------------------------------------------
local function drawHeading(d)
  local x, y, w, h = L.pfdX, L.botY, L.pfdW, L.hdgH
  fillRect(x, y, w, h, C.panel)

  local half = 45
  local ppd = w / (2 * half)
  local hdg = d.hdg
  local cxm = x + w / 2
  local tmaj, tmin = L.tickMaj, L.tickMin
  local ly = y + tmaj + 1
  local fs = L.fs
  local d0 = floor((hdg - half) / 5) * 5

  local lstep, lm, lf = L.hLblStep, L.lw3 / 2 + 2, L.lblFont
  lcd.setColor(CC, C.white)
  for a = d0, hdg + half, 5 do
    local px = cxm + (a - hdg) * ppd
    if px > x + 2 and px < x + w - 2 then
      local dd = a % 360
      local pxi = floor(px)
      if dd % 10 == 0 then
        drawLine(pxi, y, pxi, y + tmaj, SOLID, CC)
        local lbl = CARD[dd]
        if (lbl or dd % lstep == 0) and px > x + lm and px < x + w - lm then
          if lbl then
            lcd.setColor(CC, C.yellow)
            drawText(pxi, ly, lbl, lf + CENTER + CC)
            lcd.setColor(CC, C.white)
          else
            drawText(pxi, ly, fmt("%d", dd), lf + CENTER + CC)
          end
        end
      else
        drawLine(pxi, y, pxi, y + tmin, SOLID, CC)
      end
    end
  end

  -- home marker on the tape
  if d.bearing then
    local diff = ((d.bearing - hdg + 540) % 360) - 180
    if abs(diff) < half then
      local px = cxm + diff * ppd
      local m = L.sh
      fillRect(px - m / 2, y + 1, m, m, C.magenta)
      text(px, y + 1, "H", C.black, fs + CENTER)
    end
  end

  -- fixed pointer + numeric heading
  line(cxm, y, cxm, y + h, C.yellow, true)
  local bw, bh = L.w3 + 12, L.sh + 6
  local bx, by = L.cx - bw / 2, L.pfdY + L.pfdH - bh
  fillRect(bx, by, bw, bh, C.black)
  box(bx, by, bw, bh, C.white)
  text(L.cx, by + 3, fmt("%03d", I(hdg) % 360), C.green, fs + CENTER)
  box(x, y, w, h, C.grey)
end

---------------------------------------------------------------------------
-- drawing: top bar & bottom info
---------------------------------------------------------------------------
local function drawTopBar(d, warnRssi, warnLq)
  fillRect(0, 0, L.W, L.topH, C.panel)
  local ty = (L.topH - L.sh) / 2 - 1
  text(6, ty, d.mode, C.green, L.fs)
  if d.armed then
    text(L.W * 0.30, ty, "ARMED", C.green, L.fs)
  else
    text(L.W * 0.30, ty, "DISARMED", C.yellow, L.fs)
  end
  local satc = (d.sats >= 8) and C.green or ((d.sats >= 6) and C.yellow or C.red)
  text(L.W * 0.54, ty, fmt("SAT %d", I(d.sats)), satc, L.fs)

  -- Link quality and RSSI: green, or flashing red/yellow at or below the warning threshold
  -- (0 = no warning). A small hysteresis (3 points / 3 dB) keeps the warning from chattering.
  if warnLq > 0 and not d.lost then
    if d.lq <= warnLq then d.lowLq = true
    elseif d.lq > warnLq + 3 then d.lowLq = false end
  else
    d.lowLq = false
  end
  if warnRssi > 0 and not d.lost and d.rssi ~= 0 then
    local weak = -d.rssi                       -- RSSI is negative dBm; bigger = weaker
    if weak >= warnRssi then d.lowRssi = true
    elseif weak < warnRssi - 3 then d.lowRssi = false end
  else
    d.lowRssi = false
  end
  local flash = ((getTime() % 100) < 50) and C.red or C.yellow
  local lqc = d.lost and C.grey or (d.lowLq and flash or C.green)
  local rsc = d.lost and C.grey or (d.lowRssi and flash or C.green)
  text(L.W * 0.70, ty, fmt("LQ %d", I(d.lq)), lqc, L.fs)
  text(L.W - 4, ty, fmt("%ddB", I(d.rssi)), rsc, L.fs + RIGHT)
  line(0, L.topH - 1, L.W, L.topH - 1, C.grey, false)
end

-- Block arrow (head + shaft) in unit coordinates, pointing up: x, y pairs.
-- Rotated per frame with one sin/cos; outline only, 2 passes for a thicker line.
local ARROW = { 0, -1,  0.79, 0.06,  0.46, 0.06,  0.46, 1,
                -0.46, 1,  -0.46, 0.06,  -0.79, 0.06 }
local arrowX, arrowY = {}, {}

local function drawArrow(cx, cy, r, ang, c)
  local ca, sa = cos(ang), sin(ang)
  for i = 1, 7 do
    local vx, vy = ARROW[2 * i - 1], ARROW[2 * i]
    arrowX[i] = floor(cx + r * (vx * ca - vy * sa) + 0.5)
    arrowY[i] = floor(cy + r * (vx * sa + vy * ca) + 0.5)
  end
  lcd.setColor(CC, c)
  for i = 1, 7 do
    local j = (i % 7) + 1
    local x1, y1, x2, y2 = arrowX[i], arrowY[i], arrowX[j], arrowY[j]
    drawLine(x1, y1, x2, y2, SOLID, CC)
    drawLine(x1 + 1, y1 + 1, x2 + 1, y2 + 1, SOLID, CC)
  end
end

local function fmtTime(t)
  local secs = floor(t)
  if secs >= 3600 then
    return fmt("%d:%02d:%02d", secs // 3600, (secs % 3600) // 60, secs % 60)
  end
  return fmt("%02d:%02d", secs // 60, secs % 60)
end

local function drawBottom(d, imperial, warnV, warnMah)
  local y0 = L.botY
  -- panel background in three pieces: the heading tape paints its own area
  fillRect(0, y0, L.pfdX, L.botH, C.panel)
  fillRect(L.pfdX + L.pfdW, y0, L.W - L.pfdX - L.pfdW, L.botH, C.panel)
  fillRect(L.pfdX, y0 + L.hdgH, L.pfdW, L.botH - L.hdgH, C.panel)

  -- battery (left block)
  -- green above the warning voltage, flashing red/yellow at or below it.
  -- Hysteresis: once the warning has tripped it only clears when the voltage
  -- recovers 2% above the setting, so load sag around the threshold can't make it chatter.
  if warnV > 0 and d.volt > 0 then
    if d.volt <= warnV then
      d.lowBat = true
    elseif d.volt > warnV * 1.02 then
      d.lowBat = false
    end
  else
    d.lowBat = false
  end
  local bc = C.green
  if d.lowBat then
    bc = ((getTime() % 100) < 50) and C.red or C.yellow
  end
  text(5, y0 + 2, fmt("%.1fV", d.volt), bc, L.voltFont)
  text(5, y0 + 2 + L.voltH + 2, fmt("%.1fA", d.curr), C.white, L.batFont)
  local mc = C.green
  if warnMah > 0 and d.capa >= warnMah then
    mc = ((getTime() % 100) < 50) and C.red or C.yellow
  end
  text(5, y0 + 2 + L.voltH + L.batH + 4, fmt("%dmAh", I(d.capa)), mc, L.batFont)

  -- home (right block): distance at the bottom, arrow centred in the space above it
  local bx = L.pfdX + L.pfdW
  box(bx, y0, L.tapeW, L.botH, C.grey)
  if d.bearing then
    local dist = d.dist
    local s
    if imperial then
      local ft = dist * 3.281
      s = (ft < 5280) and fmt("%dft", I(ft)) or fmt("%.1fmi", ft / 5280)
    else
      s = (dist < 1000) and fmt("%dm", I(dist)) or fmt("%.1fkm", dist / 1000)
    end
    local f = (#s * L.mh * 0.55 > L.tapeW - 6) and L.fs or L.fm
    local fh = (f == L.fm) and L.mh or L.sh
    local ty = y0 + L.botH - fh - 3                 -- top of the distance text
    local space = ty - y0
    local R = math.min((space - 8) / 2, (L.tapeW / 2 - 6) / 0.79, 24 * L.k)
    if R < 7 then R = 7 end
    drawArrow(bx + L.tapeW / 2, y0 + space / 2, R, rad(d.bearing - d.hdg), C.magenta)
    text(bx + L.tapeW / 2, ty, s, C.white, f + CENTER)
  else
    local nx = bx + L.tapeW / 2
    if L.nhLines == 1 then
      text(nx, y0 + L.botH / 2 - L.nhH / 2, "NO HOME", C.grey, L.nhFont + CENTER)
    else
      local ty = y0 + L.botH / 2 - L.nhH
      text(nx, ty, "NO", C.grey, L.nhFont + CENTER)
      text(nx, ty + L.nhH, "HOME", C.grey, L.nhFont + CENTER)
    end
  end

  -- ground speed / vertical speed under the heading tape
  local ry = y0 + L.hdgH + 4
  local gs, vs = d.gs, d.vs
  local gsTxt, vsTxt
  if imperial then
    gsTxt = fmt("GS %d mph", I(gs * 0.6214))
    vsTxt = fmt("VS %+.1f ft/s", vs * 3.281)
  else
    gsTxt = fmt("GS %d km/h", I(gs))
    vsTxt = fmt("VS %+.1f m/s", vs)
  end
  text(L.pfdX + 6, ry, gsTxt, C.white, L.fs)
  text(L.pfdX + L.pfdW - 6, ry, vsTxt, C.white, L.fs + RIGHT)

  -- flight timer: white while flying, grey after landing / before the first flight
  text(L.cx, ry + L.sh / 2 - L.timerH / 2, fmtTime(d.flightSecs or 0),
       d.flying and C.white or C.grey, L.timerFont + CENTER)
end

---------------------------------------------------------------------------
-- widget API
---------------------------------------------------------------------------
-- warning voltage typed as text ("12.2", "12,2" and "12.2V" all work); empty or bad input -> 0 (off)
local function parseWarn(opts)
  local str = tostring(opts.WarnVolts or "0")
  str = string.gsub(str, ",", ".")
  local m = string.match(str, "%d+%.?%d*")
  local v = m and tonumber(m)
  if v == nil then v = 0 end
  return v
end

local function create(zone, opts)
  initColors()
  computeLayout()
  return { zone = zone, options = opts, d = {}, home = nil,
           hLat = {}, hLon = {}, hTime = {},
           warnV = parseWarn(opts), warnLq = opts.WarnLQ or 0, warnRssi = opts.WarnRSSI or 0,
           warnMah = opts.WarnMAh or 0 }
end

local function update(w, opts)
  w.options = opts
  w.warnV = parseWarn(opts)
  w.warnLq, w.warnRssi = opts.WarnLQ or 0, opts.WarnRSSI or 0
  w.warnMah = opts.WarnMAh or 0
  computeLayout()
end

local function background(w)
  poll(w)
end

-- shown on the ground after a flight, until the next arming
local function drawSummary(d, imperial)
  local lh = L.sumH + 4
  local bw = L.sumW
  local bh = L.sh + 7 * lh + 16
  local bx, by = L.cx - bw / 2, L.cy - bh / 2

  local alt, altU = d.sumAlt or 0, "m"
  local spd, spdU = d.sumSpd or 0, "km/h"
  local dist = d.sumDist or 0
  local dtxt
  if imperial then
    alt, altU = alt * 3.281, "ft"
    spd, spdU = spd * 0.6214, "mph"
    local ft = dist * 3.281
    dtxt = (ft < 5280) and fmt("%d ft", I(ft)) or fmt("%.1f mi", ft / 5280)
  else
    dtxt = (dist < 1000) and fmt("%d m", I(dist)) or fmt("%.1f km", dist / 1000)
  end

  fillRect(bx, by, bw, bh, C.black)
  box(bx, by, bw, bh, C.grey)
  text(L.cx, by + 6, "FLIGHT SUMMARY", C.grey, L.fs + CENTER)

  local function row(i, label, value)
    local ty = by + 6 + L.sh + 6 + (i - 1) * lh
    text(bx + 10, ty + (L.sumH - L.sh) / 2, label, C.grey, L.fs)
    text(bx + bw - 10, ty, value, C.white, L.sumFont + RIGHT)
  end
  row(1, "TIME", fmtTime(d.flightSecs or 0))
  row(2, "MAX ALT", fmt("%d %s", I(alt), altU))
  row(3, "MAX SPD", fmt("%d %s", I(spd), spdU))
  row(4, "MAX DIST", dtxt)
  row(5, "MIN V", d.sumMinV and fmt("%.1f V", d.sumMinV) or "--")
  row(6, "MAX A", fmt("%.1f A", d.sumMaxA or 0))
  row(7, "USED", fmt("%d mAh", I(d.sumUsed or 0)))
end

local function refresh(w, event, touchState)
  if w.zone.w < LCD_W * 0.9 or w.zone.h < LCD_H * 0.9 then
    lcd.drawText(w.zone.x + 4, w.zone.y + 4, "Set Glass to full screen", SMLSIZE)
    return
  end

  poll(w)
  local d = w.d
  local imperial = (w.options.Imperial ~= 0)

  drawHorizon(d)
  drawTapes(d, imperial)
  drawTopBar(d, w.warnRssi, w.warnLq)
  drawBottom(d, imperial, w.warnV, w.warnMah)
  drawHeading(d)          -- after the bottom panel, which fills the area under it
  if d.showSummary and not d.flying and not d.lost then drawSummary(d, imperial) end

  if d.lost then
    local bw = math.min(L.pfdW - 10, L.wTel + 20)
    local bh = L.mh + 8
    fillRect(L.cx - bw / 2, L.cy - bh / 2, bw, bh, C.black)
    box(L.cx - bw / 2, L.cy - bh / 2, bw, bh, C.red)
    text(L.cx, L.cy - L.mh / 2 - 1, "NO TELEMETRY", C.red, L.fm + CENTER)
  end
end

return {
  name = "Glass",
  options = options,
  create = create,
  update = update,
  refresh = refresh,
  background = background
}
