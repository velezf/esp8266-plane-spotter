/*
 * Configuration template.
 *
 *   1. Copy this file to "config.h" (same folder).
 *   2. Fill in your WiFi credentials and home coordinates.
 *   3. config.h is git-ignored so your secrets never get committed.
 */
#pragma once

// ---- WiFi ----------------------------------------------------------------
#define WIFI_SSID  "YOUR_WIFI_NAME"
#define WIFI_PASS  "YOUR_WIFI_PASSWORD"

// ---- Home location -------------------------------------------------------
// Default: centre of Pinerolo (TO), Italy.
#define HOME_LAT   44.8848
#define HOME_LON   7.3306

// Half-size of the search box in degrees (~1.0 deg latitude is ~111 km).
// Smaller = less data to parse on the ESP8266, larger = wider sky coverage
// (but more RAM used; on a quiet sky raise it, if it reboots lower it).
#define SEARCH_RADIUS_DEG  1.0

// How often to poll OpenSky, in milliseconds. Anonymous access has a small
// daily budget (~400 calls): 60 s already exceeds it over 24 h, so for
// continuous use set OPENSKY_USER/PASS below (a free account = far more calls).
#define UPDATE_INTERVAL_MS  60000

// Compass heading (deg, 0=N 90=E 180=S 270=W) that the wall / device faces.
// Used by the radar screen to mark the wall direction.
#define WALL_HEADING_DEG  194

// POSIX timezone string for the NTP clock (default: Europe/Rome). See
// https://github.com/nayarsystems/posix_tz_db for other zones.
#define TIMEZONE  "CET-1CEST,M3.5.0,M10.5.0/3"

// How often to refresh weather from Open-Meteo (ms). 10 min is plenty.
#define WEATHER_INTERVAL_MS  600000

// ---- OpenSky OAuth2 client (optional) ------------------------------------
// Leave both empty for anonymous access (small ~400 calls/day budget). For a
// much larger quota, create a free account at https://opensky-network.org/,
// then Account -> API clients -> create a client, and paste the resulting
// clientId / clientSecret here. The firmware fetches an OAuth2 token and sends
// it as a Bearer header.
#define OPENSKY_CLIENT_ID      ""
#define OPENSKY_CLIENT_SECRET  ""

// ---- Rotorcraft loiter detection -----------------------------------------
// A helicopter that stays within LOITER_RADIUS_KM of where it was first seen
// for LOITER_MIN_MS is "loitering" (orbiting traffic/news/survey birds), as
// opposed to one transiting through. Drift outside the radius re-anchors it.
// Raise the radius if wide orbits keep resetting; lower it to only catch tight
// holds. HELI_EXPIRE_MS is how long a helicopter is remembered after it drops
// off ADS-B -- kept generously longer than LOITER_MIN_MS so a brief ADS-B
// coverage gap does not reset an orbit that is already accumulating.
//
// Detection is sampled at the poll rate, so LOITER_MIN_MS is really "this many
// UPDATE_INTERVAL_MS samples in a row". Two samples is the practical floor:
// one sample decides off a single displacement, and a helicopter cruising
// ~200 km/h only just clears the 3 km radius in 60 s, so a slower transit
// would false-positive. It matters more than it looks, because categoryFrom()
// *guesses* rotorcraft for anything under 120 km/h when OpenSky omits the
// category -- those are slow by definition and would latch instantly at one
// sample. By two samples even a 120 km/h target is 4 km out and re-anchors.
//
// To detect faster you have to change the input, not this number: either poll
// harder (UPDATE_INTERVAL_MS, costs OpenSky quota) or discriminate on track
// swing rather than displacement, since an orbiting aircraft sweeps heading
// through 360 deg while a transit holds it steady.
#define LOITER_RADIUS_KM  3.0
#define LOITER_MIN_MS     120000   // 2 min (= 2 polls at the default 60 s)
#define HELI_EXPIRE_MS    300000   // 5 min

// ---- Piezo buzzer (rotorcraft alerts) ------------------------------------
// Passive buzzer -- it has no oscillator of its own, so the firmware drives it
// with tone(). 3-pin module: VCC->3V3, GND->GND, S/IO->PIN_BUZZER. Bare 2-pin
// element: one leg to PIN_BUZZER, the other to GND, and set BUZZER_ACTIVE_LOW
// to 0. For a bare element a ~100 ohm series resistor is cheap insurance --
// ESP8266 pins are only good for ~12 mA; transistor modules drive their own
// current from VCC and do not need it.
//
// D6/GPIO12 is the right pin here and the alternatives mostly are not:
// GPIO16 (D0) is not on the normal GPIO mux and cannot do tone(); GPIO0 (D3)
// and GPIO2 (D4) must be HIGH at boot and a buzzer coil dragging them down
// stops the board booting; GPIO15 (D8) must be LOW at boot. GPIO12 has no
// strapping role and is free in both the I2C and SPI display builds.
#define PIN_BUZZER  12   // D6

// Master switch. 0 compiles the buzzer out entirely.
#define BUZZER_ENABLE  1

// Drive polarity. 3-pin modules built around an S9012 (a PNP transistor) sound
// when the signal pin is pulled LOW, because a PNP conducts on a low base --
// so idle has to be HIGH. This matters more than it sounds: the ESP8266 core's
// noTone() finishes with digitalWrite(pin, 0), which on such a module leaves
// the buzzer howling after every chirp, so the idle level is restored by hand.
// Set to 0 for a bare 2-pin piezo or an NPN-driven module (idle LOW).
#define BUZZER_ACTIVE_LOW  1

// Only sound off for rotorcraft closer than this. The radar ring is 30 km, so
// something well inside that is the interesting case.
#define BUZZER_RANGE_KM  15.0

// Radar-sweep blip: chirp as the sweep passes over a rotorcraft, but only
// while the RADAR screen is actually showing. That works out to ~3 chirps per
// screen cycle rather than a continuous sonar, which is what makes it
// tolerable. Set to 0 to keep only the acquisition/loiter alerts.
#define BUZZER_SWEEP_BLIP  1

// Quiet hours (local time, 24 h). A helicopter at 3 am should not wake you.
// Set both to the same value to disable. Wraps midnight, so 22 -> 5 works.
#define BUZZER_QUIET_START  22
#define BUZZER_QUIET_END     5

// ---- WEAPONS SYSTEM page -------------------------------------------------
// A themed air-defense reference display over the ADS-B data already fetched.
// Every contact is treated as FRIENDLY and firing authorization is always
// denied (WEAPONS HOLD) -- the page classifies and displays, it does not
// engage anything and is not connected to anything.
//
// Envelope and time-of-flight come from approximate published reference
// figures for each system. PK is a transparent geometric heuristic, not a real
// lethality estimate -- there is no public data from which one could be
// computed, so it is labelled NOTIONAL on screen.

// Look-ahead used by the SOLUTION track-lead angle.
#define TRACK_LOOKAHEAD_SECONDS  30

// Below this ground speed a track has no meaningful heading, so no solution.
#define TRACK_MIN_SPEED_MS  10.0

// How old the last fetch may be before the solution blanks to "---.-".
// NOTE: deliberately larger than UPDATE_INTERVAL_MS (60 s). The obvious 30 s
// would mark the data stale for the second half of every single poll cycle,
// so the page would show "---.-" roughly half the time on a working device.
// 1.5 poll intervals means one missed fetch blanks it, which is the intent.
#define TRACK_STALE_MS  90000

// Which service's reference systems to prefer. 0 = Army, 1 = Navy, 2 = Marine
// expeditionary site.
#define DEFENSE_THEME_ARMY    0
#define DEFENSE_THEME_NAVY    1
#define DEFENSE_THEME_MARINE  2
#define DEFENSE_THEME  DEFENSE_THEME_MARINE

// THREAT gates. Aspect is how directly the contact tracks over the device
// (0 deg = straight at you); range is SLANT range, so altitude counts -- a jet
// at FL350 overhead is 10.7 km away and will never rate HIGH, which is the
// point. HIGH additionally requires a known altitude.
//
// HIGH is meant to mean "close enough to read markings". Note the floor is set
// by the poll rate, not by eyesight: true naked-eye tail-number range is more
// like 0.3-0.5 km, but at 250 km/h an aircraft crosses a 0.5 km bubble in ~14 s,
// so a 60 s poll would miss it roughly three times in four. 3 km keeps it
// inside for ~1.4 min, which one sample per minute reliably catches. Tighten
// it if you would rather have rare-but-accurate than dependable.
// Slant range alone is not enough for HIGH: a 3 km slant admits anything below
// ~9800 ft when it is directly overhead, which is far past reading markings.
// This cap is what actually enforces "low", and the slant gate then enforces
// "close". Both must hold.
#define THREAT_HIGH_MAX_FT      5000
#define THREAT_HIGH_SLANT_KM     3.0
#define THREAT_HIGH_ASPECT_DEG  30.0
#define THREAT_MED_SLANT_KM     10.0
#define THREAT_MED_ASPECT_DEG   60.0

// Altitude band edges (feet). UI categories only -- not engagement limits.
// VERY LOW exists because a device sitting under an approach path sees most of
// its traffic below 10000 ft, and one band there discriminates nothing.
#define ALT_VLOW_MAX_FT    5000
#define ALT_LOW_MAX_FT    10000
#define ALT_MED_MAX_FT    30000
#define ALT_HIGH_MAX_FT   60000

// ---- OLED wiring (I2C, 4-pin panel -- the default build) -----------------
// NON-STANDARD PINS: D7/D5 instead of the ESP8266 defaults D2/D1, because the
// 7-pin SPI panel this project started with already had wires on those two
// pads. Software I2C on the ESP8266 does not care which GPIOs are used.
#define PIN_OLED_SDA  13   // D7
#define PIN_OLED_SCL  14   // D5

// 7-bit address. Most 0.96" SSD1306 modules are 0x3C, a few are 0x3D; setup()
// probes for both, so this is only the preferred one to try first.
#define OLED_I2C_ADDR      0x3C
#define OLED_I2C_ADDR_ALT  0x3D

// ---- OLED wiring (hardware SPI, only used when DISPLAY_I2C is 0) ---------
// SCK -> GPIO14 (D5) and SDA/MOSI -> GPIO13 (D7) are fixed by HW SPI.
// These three are configurable:
#define PIN_OLED_RST  16   // D0
#define PIN_OLED_DC    4   // D2
#define PIN_OLED_CS    5   // D1
