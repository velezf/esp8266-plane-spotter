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
