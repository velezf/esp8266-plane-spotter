/*
 * ESP8266 Plane Spotter
 * --------------------------------------------------------------------------
 * Shows the aircraft currently flying closest to your home on a 0.96" SSD1306
 * I2C OLED, plus a bunch of nerdy statistics. Live ADS-B data is pulled from
 * the free OpenSky Network REST API.
 *
 * Board   : any ESP8266 (NodeMCU v2/v3, Wemos D1 mini, ...)
 * Display : 0.96" OLED I2C, SSD1306 128x64 (4 pins: GND, VCC, SCL, SDA)
 *
 * Wiring (default, see README for the full table):
 *   OLED      ESP8266 (NodeMCU label / GPIO)
 *   GND  -->  GND
 *   VCC  -->  3V3
 *   SCL  -->  D5  / GPIO14   (NON-STANDARD I2C clock, see note below)
 *   SDA  -->  D7  / GPIO13   (NON-STANDARD I2C data,  see note below)
 *
 * The 7-pin SPI panel this project originally used is still supported: set
 * DISPLAY_I2C to 0 below and wire SCK=D5, SDA=D7, RES=D0, DC=D2, CS=D1.
 *
 * Libraries (install from the Arduino Library Manager):
 *   - U8g2        by olikraus
 *   - ArduinoJson by Benoit Blanchon (v6.x)
 *
 * Copy config.example.h to config.h and fill in your details before flashing.
 */

#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>

#include "config.h"

// ---------------------------------------------------------------------------
// Display: SSD1306 128x64. Two builds selectable with DISPLAY_I2C:
//   1 = I2C, 4-pin panel (default). GND / VCC / SCL / SDA only.
//   0 = 4-wire hardware SPI, 7-pin panel (the original build).
// Everything below the constructor is bus-agnostic U8g2 API, so only the
// constructor changes between the two builds.
//
// NON-STANDARD I2C PINS: this board is wired SDA=GPIO13 (D7) and SCL=GPIO14
// (D5), NOT the ESP8266 defaults SDA=GPIO4 (D2) / SCL=GPIO5 (D1). Those are
// the two pins the 7-pin SPI panel already used (HW SPI MOSI/SCLK), so the
// existing solder joints were reused as-is. This is fine on the ESP8266: the
// Arduino Wire library here is a bit-banged software I2C master, so any pair
// of GPIOs works as long as the pins are passed explicitly to Wire.begin() --
// which the U8g2 HW_I2C backend does for us when the constructor is given a
// clock/data pair (see U8x8lib.cpp, U8X8_MSG_BYTE_INIT).
//
// The panel has no reset line, so reset is U8X8_PIN_NONE (the U8g2 equivalent
// of passing -1 for the reset pin to Adafruit_SSD1306).
//
// Address: most 0.96" modules are 0x3C, some are 0x3D. setup() probes both and
// picks whichever ACKs, so either module works without a recompile. Note U8g2
// takes the *8-bit* address, i.e. 0x3C -> 0x78 and 0x3D -> 0x7A.
// ---------------------------------------------------------------------------
#define DISPLAY_I2C 1   // flip to 0 and reflash if you swap back to an SPI panel

#if DISPLAY_I2C
// Constructor args: (rotation, reset, clock/SCL, data/SDA).
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0,
                                         /*reset=*/ U8X8_PIN_NONE,
                                         /*clock=*/ PIN_OLED_SCL,
                                         /*data=*/  PIN_OLED_SDA);
#else
U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0,
                                            PIN_OLED_CS,
                                            PIN_OLED_DC,
                                            PIN_OLED_RST);
#endif

// ---------------------------------------------------------------------------
// Data model
// ---------------------------------------------------------------------------
struct Aircraft {
  char   icao24[8];
  char   callsign[10];
  char   country[24];
  double lat;
  double lon;
  float  altitudeM;   // geometric/barometric altitude in metres
  float  velocityMs;  // ground speed in m/s
  float  trackDeg;    // true track over ground (0 = north)
  float  vrateMs;     // vertical rate in m/s (+climb / -descent)
  bool   onGround;
  int    category;    // OpenSky emitter category (state index 17, 0 = unknown)
  double distanceKm;  // great-circle distance from home
  double bearingDeg;  // bearing from home to aircraft
  bool   valid;
};

Aircraft nearest;

// Types for the WEAPONS SYSTEM page. Declared up here because the Arduino
// builder emits function prototypes above the sketch body, so anything used in
// a signature must already be a complete type by then.
enum class AirframeClass : uint8_t { FIXED_WING, HELICOPTER, UAV, UNKNOWN };
enum class AltitudeBand  : uint8_t { VERY_LOW, LOW_ALT, MEDIUM, MED_HIGH, HIGH_ALT, UNKNOWN };
enum class ThreatLevel   : uint8_t { LOW_THREAT, MED_THREAT, HIGH_THREAT, UNKNOWN };
enum class Envelope      : uint8_t { INSIDE, TOO_FAR, TOO_CLOSE, ALT_OUT, NO_DATA };

// Approximate *published* reference figures. These are open-source
// encyclopaedia-level numbers for flavour and are not authoritative: no
// doctrinal engagement rules or no-escape-zone data are represented, because
// none is publicly available to represent honestly.
struct WeaponSystemRecord {
  const char* designation;
  const char* name;
  const char* branch;
  const char* role;
  uint16_t    maxRangeKm;
  uint16_t    minRangeKm;
  uint16_t    ceilingKft;      // engagement ceiling, thousands of feet
  uint16_t    missileSpeedMps;
  uint8_t     reactionS;       // notional time from track to launch
  uint8_t     preferredAirframe;
};


// Per-aircraft state for the radar, including enough to dead-reckon (estimate)
// the position between data refreshes so the blips creep in real time. `cat`
// and `loiter` are resolved at fetch time so the radar can mark rotorcraft
// without re-deriving the type 30x a second.
struct Blip {
  double  lat;
  double  lon;
  float   track;
  float   speedMs;
  uint8_t cat;      // effective emitter category (8 = rotorcraft)
  bool    loiter;   // rotorcraft that has held station (see HeliTrack)
};
const uint8_t MAX_BLIPS = 20;
Blip     blips[MAX_BLIPS];
uint8_t  blipCount  = 0;
uint32_t lastDataMs = 0;   // millis() of the last successful aircraft fetch

// Rotorcraft get special treatment, which means they need an identity that
// survives across fetches -- blips[] is rebuilt from scratch every poll, so it
// cannot answer "has this one been sitting there?". This table is keyed by
// icao24 and holds an anchor position per helicopter: stay within
// LOITER_RADIUS_KM of the anchor for LOITER_MIN_MS and it is loitering; wander
// outside and the anchor resets, because that is transit, not orbit.
struct HeliTrack {
  char     icao24[7];      // 6 hex chars + NUL
  double   refLat, refLon; // anchor position
  uint32_t sinceMs;        // when this anchor was set
  uint32_t lastSeenMs;
  bool     loitering;
};
const uint8_t MAX_HELI = 4;
HeliTrack helis[MAX_HELI];
uint8_t   heliCount = 0;

// OpenSky OAuth2 bearer token (when client credentials are configured).
String   accessToken;
uint32_t tokenExpiryMs = 0;

// Current weather (from Open-Meteo).
struct Weather {
  float tempC;
  float windKmh;
  int   humidity;
  int   code;      // WMO weather code
  bool  valid = false;
} weather;
uint32_t lastWeatherPoll = 0;

// Short hourly forecast (a few hours ahead).
struct Fcast { int hour; float tempC; int code; };
const uint8_t FC_N = 3;
Fcast   fcast[FC_N];
uint8_t fcCount = 0;

// Route / airline / ETA for the current nearest aircraft (from hexdb.io).
struct RouteInfo {
  char   callsign[10];   // which callsign this data is for
  char   airline[18];
  char   dep[6];
  char   arr[6];
  bool   haveRoute;
  bool   haveArrPos;
  double arrLat, arrLon;
} routeInfo;

// Runtime statistics (the nerdy bit)
struct Stats {
  uint32_t requestsOk   = 0;
  uint32_t requestsFail = 0;
  uint16_t inView       = 0;   // aircraft inside the search box on last poll
  uint16_t maxInView    = 0;   // session record
  double   closestEver  = 1e9; // closest distance seen this session (km)
  uint32_t lastUpdateMs = 0;
} stats;

uint8_t  screen           = 0;         // which screen is showing
uint32_t lastScreenSwap   = 0;
uint32_t lastPoll         = 0;
bool     firstFetchDone   = false;
bool     firstWeatherDone = false;

// Page order: RADAR opens with the situational picture, then TARGET / INTEL /
// WEAPONS are three progressively deeper views of that same nearest contact,
// then the ambient pages. Naming them keeps the dwell table and the
// index-sensitive logic in loop() from drifting apart on the next reorder --
// add or move a page here and everything else follows.
enum Screen : uint8_t {
  SCR_RADAR, SCR_TARGET, SCR_INTEL, SCR_WEAPONS, SCR_WX, SCR_SYSTEM, NUM_SCREENS
};

// Per-screen dwell time (ms), in the same order as `enum Screen`.
const uint32_t SCREEN_SWAP_MS[NUM_SCREENS] = {
  12000,  // RADAR
  12000,  // TARGET
  12000,  // INTEL
  12000,  // WEAPONS
   7000,  // WX
   7000,  // SYSTEM
};

// ---------------------------------------------------------------------------
// Geo helpers
// ---------------------------------------------------------------------------
static double deg2rad(double d) { return d * (PI / 180.0); }
static double rad2deg(double r) { return r * (180.0 / PI); }

// Great-circle distance (Haversine) in kilometres.
double haversineKm(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371.0;
  double dLat = deg2rad(lat2 - lat1);
  double dLon = deg2rad(lon2 - lon1);
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(deg2rad(lat1)) * cos(deg2rad(lat2)) *
             sin(dLon / 2) * sin(dLon / 2);
  return R * 2 * atan2(sqrt(a), sqrt(1 - a));
}

// Initial bearing from point 1 to point 2, degrees 0..360.
double bearingDeg(double lat1, double lon1, double lat2, double lon2) {
  double y = sin(deg2rad(lon2 - lon1)) * cos(deg2rad(lat2));
  double x = cos(deg2rad(lat1)) * sin(deg2rad(lat2)) -
             sin(deg2rad(lat1)) * cos(deg2rad(lat2)) * cos(deg2rad(lon2 - lon1));
  double b = rad2deg(atan2(y, x));
  return fmod(b + 360.0, 360.0);
}

const char* compass(double bearing) {
  static const char* dirs[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  return dirs[(int)((bearing + 22.5) / 45.0) % 8];
}

// ---------------------------------------------------------------------------
// WiFi
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// Aircraft type + rotorcraft tracking
// ---------------------------------------------------------------------------

// OpenSky usually leaves the emitter category at 0 (unknown), so fall back to a
// rough guess from altitude + ground speed. Real category data always wins.
// Estimated types are flagged with '~' on screen.
int categoryFrom(int cat, bool onGround, float velocityMs, float altitudeM) {
  if (cat > 0)  return cat;                   // real data
  if (onGround) return 0;
  float kmh = velocityMs * 3.6f;
  if (kmh < 120 && altitudeM < 2200) return 8;  // slow & low -> guess helicopter
  if (kmh < 300 && altitudeM < 5000) return 3;  // medium      -> guess small plane
  return 4;                                     // fast / high -> airliner
}

bool isRotor(int cat) { return cat == 8; }

// Fold one rotorcraft sighting into the tracking table. Returns true if this
// airframe currently counts as loitering. Called once per rotorcraft per fetch.
bool trackRotorcraft(const char* icao, double lat, double lon) {
  uint32_t now = millis();
  if (icao == nullptr || icao[0] == '\0') return false;

  for (uint8_t i = 0; i < heliCount; i++) {
    if (strcmp(helis[i].icao24, icao) != 0) continue;
    helis[i].lastSeenMs = now;
    if (haversineKm(helis[i].refLat, helis[i].refLon, lat, lon) > LOITER_RADIUS_KM) {
      helis[i].refLat    = lat;    // moved on: re-anchor, it is transiting
      helis[i].refLon    = lon;
      helis[i].sinceMs   = now;
      helis[i].loitering = false;
    } else if (!helis[i].loitering &&
               (uint32_t)(now - helis[i].sinceMs) >= LOITER_MIN_MS) {
      helis[i].loitering = true;
      Serial.printf("[heli] %s loitering: %lu min within %.1f km\n",
                    icao, (unsigned long)((now - helis[i].sinceMs) / 60000UL),
                    (double)LOITER_RADIUS_KM);
      if (haversineKm(HOME_LAT, HOME_LON, lat, lon) <= BUZZER_RANGE_KM)
        buzzerLoiter();
    }
    return helis[i].loitering;
  }

  // New airframe. If the table is full, evict the stalest entry -- a helicopter
  // we have not seen in a while is less interesting than one on screen now.
  uint8_t slot;
  if (heliCount < MAX_HELI) {
    slot = heliCount++;
  } else {
    slot = 0;
    for (uint8_t i = 1; i < heliCount; i++)
      if ((uint32_t)(now - helis[i].lastSeenMs) >
          (uint32_t)(now - helis[slot].lastSeenMs)) slot = i;
  }
  strncpy(helis[slot].icao24, icao, sizeof(helis[slot].icao24) - 1);
  helis[slot].icao24[sizeof(helis[slot].icao24) - 1] = '\0';
  helis[slot].refLat     = lat;
  helis[slot].refLon     = lon;
  helis[slot].sinceMs    = now;
  helis[slot].lastSeenMs = now;
  helis[slot].loitering  = false;
  Serial.printf("[heli] new contact %s\n", icao);
  if (haversineKm(HOME_LAT, HOME_LON, lat, lon) <= BUZZER_RANGE_KM)
    buzzerAcquire();
  return false;
}

// Drop helicopters we have not heard from in a while, so a departed aircraft
// does not keep its slot (or come back still flagged as loitering).
void expireRotorcraft() {
  uint32_t now = millis();
  uint8_t  w   = 0;
  for (uint8_t i = 0; i < heliCount; i++) {
    if ((uint32_t)(now - helis[i].lastSeenMs) < HELI_EXPIRE_MS) {
      if (w != i) helis[w] = helis[i];
      w++;
    } else {
      Serial.printf("[heli] %s lost\n", helis[i].icao24);
    }
  }
  heliCount = w;
}

bool isLoitering(const char* icao) {
  if (icao == nullptr || icao[0] == '\0') return false;
  for (uint8_t i = 0; i < heliCount; i++)
    if (strcmp(helis[i].icao24, icao) == 0) return helis[i].loitering;
  return false;
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint8_t dots = 0;
  while (WiFi.status() != WL_CONNECTED) {
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tr);
    u8g2.drawStr(0, 12, "Plane Spotter");
    u8g2.drawStr(0, 30, "Connecting WiFi");
    u8g2.setCursor(0, 46);
    u8g2.print(WIFI_SSID);
    u8g2.setCursor(0, 62);
    for (uint8_t i = 0; i < (dots % 16) + 1; i++) u8g2.print('.');
    u8g2.sendBuffer();
    delay(400);
    dots++;
  }
  Serial.printf("\n[wifi] connected SSID=%s IP=%s RSSI=%d\n",
                WIFI_SSID, WiFi.localIP().toString().c_str(), WiFi.RSSI());
}

// ---------------------------------------------------------------------------
// OpenSky fetch
// ---------------------------------------------------------------------------
String buildUrl() {
  double lamin = HOME_LAT - SEARCH_RADIUS_DEG;
  double lamax = HOME_LAT + SEARCH_RADIUS_DEG;
  double lomin = HOME_LON - SEARCH_RADIUS_DEG;
  double lomax = HOME_LON + SEARCH_RADIUS_DEG;

  String url = "https://opensky-network.org/api/states/all?";
  url += "lamin=" + String(lamin, 4);
  url += "&lomin=" + String(lomin, 4);
  url += "&lamax=" + String(lamax, 4);
  url += "&lomax=" + String(lomax, 4);
  url += "&extended=1";   // include the aircraft category (state index 17)
  return url;
}

// Whether OAuth2 client credentials are configured.
bool oauthConfigured() { return strlen(OPENSKY_CLIENT_ID) > 0; }

// Request a fresh OAuth2 access token (client_credentials grant). Returns true
// on success and stores it in `accessToken` with an expiry a minute early.
bool fetchToken() {
  WiFiClientSecure client;
  client.setInsecure();
  client.setBufferSizes(16384, 512);

  HTTPClient https;
  https.setReuse(false);
  if (!https.begin(client,
        "https://auth.opensky-network.org/auth/realms/opensky-network/protocol/openid-connect/token")) {
    Serial.println("[auth] begin() failed");
    return false;
  }
  https.addHeader("Content-Type", "application/x-www-form-urlencoded");
  String body = "grant_type=client_credentials&client_id=";
  body += OPENSKY_CLIENT_ID;
  body += "&client_secret=";
  body += OPENSKY_CLIENT_SECRET;

  int code = https.POST(body);
  Serial.printf("[auth] token HTTP %d\n", code);
  if (code != HTTP_CODE_OK) { https.end(); return false; }

  String payload = https.getString();
  https.end();

  JsonDocument filter;
  filter["access_token"] = true;
  filter["expires_in"]   = true;
  JsonDocument doc;
  if (deserializeJson(doc, payload, DeserializationOption::Filter(filter))) {
    Serial.println("[auth] token JSON parse failed");
    return false;
  }
  const char* tok = doc["access_token"] | "";
  if (!tok[0]) return false;

  accessToken = tok;
  int exp = doc["expires_in"] | 1800;
  tokenExpiryMs = millis() + (uint32_t)(exp > 120 ? exp - 60 : exp) * 1000UL;
  Serial.printf("[auth] token ok (len=%u, expires in %ds)\n", accessToken.length(), exp);
  return true;
}

// Pulls aircraft states, keeps the nearest one. Returns true on success.
bool fetchAircraft() {
  // Refresh the OAuth2 token first, before the data client exists, so we never
  // hold two 16 KB TLS buffers at once.
  if (oauthConfigured() &&
      (accessToken.length() == 0 || (int32_t)(millis() - tokenExpiryMs) >= 0)) {
    fetchToken();
  }

  WiFiClientSecure client;
  client.setInsecure();                 // skip cert validation (read-only data)
  // Do NOT shrink the RX buffer below the default 16 KB: OpenSky does not
  // negotiate a smaller TLS fragment (no MFLN), so a small RX buffer makes the
  // TLS handshake fail and every fetch silently returns "no aircraft".
  client.setBufferSizes(16384, 512);

  HTTPClient https;
  https.setReuse(false);
  String url = buildUrl();
#if LOG_REQUEST_URL
  Serial.printf("[fetch] heap=%u GET %s\n", ESP.getFreeHeap(), url.c_str());
#else
  // Deliberately omits the bounding box -- it is centred on the configured home
  // position, so logging it would publish the device's location every fetch.
  Serial.printf("[fetch] heap=%u GET states/all (%.1f deg box)\n",
                ESP.getFreeHeap(), (double)(SEARCH_RADIUS_DEG * 2.0));
#endif
  if (!https.begin(client, url)) {
    Serial.println("[fetch] https.begin() failed");
    stats.requestsFail++;
    return false;
  }

  if (oauthConfigured() && accessToken.length() > 0) {
    https.addHeader("Authorization", "Bearer " + accessToken);
  }

  int code = https.GET();
  Serial.printf("[fetch] HTTP %d\n", code);
  if (code != HTTP_CODE_OK) {
    if (code == HTTP_CODE_UNAUTHORIZED) accessToken = "";  // force token refresh
    https.end();
    stats.requestsFail++;
    return false;
  }

  // Filter: keep only the fields we actually use from every state vector.
  // OpenSky state indices: 0 icao24, 1 callsign, 2 origin_country,
  // 5 longitude, 6 latitude, 7 baro_altitude, 8 on_ground, 9 velocity,
  // 10 true_track, 11 vertical_rate, 13 geo_altitude.
  JsonDocument filter;
  JsonArray el = filter["states"].to<JsonArray>().add<JsonArray>();
  for (int i = 0; i <= 17; i++) el[i] = false;
  el[0] = el[1] = el[2] = el[5] = el[6] = true;
  el[7] = el[8] = el[9] = el[10] = el[11] = el[13] = el[17] = true;

  // OpenSky replies with Transfer-Encoding: chunked. getStream() would hand the
  // raw chunked bytes (hex length markers) to the parser and yield nothing, so
  // we use getString(), which de-chunks the body before we parse it.
  String payload = https.getString();
  https.end();
  Serial.printf("[fetch] payload=%u bytes\n", payload.length());

  JsonDocument doc;
  DeserializationError err = deserializeJson(
      doc, payload, DeserializationOption::Filter(filter));

  if (err) {
    Serial.printf("[fetch] JSON error: %s\n", err.c_str());
    stats.requestsFail++;
    return false;
  }

  JsonArray states = doc["states"].as<JsonArray>();
  Aircraft best;
  best.valid      = false;
  best.distanceKm = 1e9;
  uint16_t count     = 0;
  uint8_t  rotorSeen = 0;
  blipCount = 0;

  for (JsonArray s : states) {
    if (s.isNull() || s[5].isNull() || s[6].isNull()) continue;
    double lon = s[5].as<double>();
    double lat = s[6].as<double>();
    double d   = haversineKm(HOME_LAT, HOME_LON, lat, lon);
    double brg = bearingDeg(HOME_LAT, HOME_LON, lat, lon);
    count++;

    // Resolve the type here, while the full state vector is in hand: the radar
    // redraws far too often to re-derive it per frame.
    bool  onGround = s[8] | false;
    float velMs    = s[9] | 0.0f;
    float altM     = s[13].isNull() ? (s[7] | 0.0f) : s[13].as<float>();
    int   cat      = categoryFrom(s[17] | 0, onGround, velMs, altM);

    bool loiter = false;
    if (isRotor(cat)) {
      rotorSeen++;
      loiter = trackRotorcraft(s[0] | "", lat, lon);
    }

    if (blipCount < MAX_BLIPS) {
      blips[blipCount].lat     = lat;
      blips[blipCount].lon     = lon;
      blips[blipCount].track   = s[10] | 0.0f;
      blips[blipCount].speedMs = onGround ? 0.0f : velMs;
      blips[blipCount].cat     = (uint8_t)cat;
      blips[blipCount].loiter  = loiter;
      blipCount++;
    }

    if (d < best.distanceKm) {
      best.distanceKm = d;
      best.lat        = lat;
      best.lon        = lon;
      best.bearingDeg = brg;
      best.onGround   = s[8] | false;
      best.category   = s[17] | 0;

      // geo altitude (13) preferred, fall back to barometric (7)
      best.altitudeM  = s[13].isNull() ? (s[7] | 0.0f) : s[13].as<float>();
      best.velocityMs = s[9]  | 0.0f;
      best.trackDeg   = s[10] | 0.0f;
      best.vrateMs    = s[11] | 0.0f;

      const char* cs = s[1] | "";
      strncpy(best.callsign, cs, sizeof(best.callsign) - 1);
      best.callsign[sizeof(best.callsign) - 1] = '\0';
      // trim trailing spaces OpenSky pads callsigns with
      for (int i = strlen(best.callsign) - 1; i >= 0 && best.callsign[i] == ' '; i--)
        best.callsign[i] = '\0';
      if (best.callsign[0] == '\0') strcpy(best.callsign, "(no id)");

      const char* ic = s[0] | "";
      strncpy(best.icao24, ic, sizeof(best.icao24) - 1);
      best.icao24[sizeof(best.icao24) - 1] = '\0';

      const char* co = s[2] | "?";
      strncpy(best.country, co, sizeof(best.country) - 1);
      best.country[sizeof(best.country) - 1] = '\0';

      best.valid = true;
    }
  }

  expireRotorcraft();

  stats.inView       = count;
  if (count > stats.maxInView) stats.maxInView = count;
  stats.requestsOk++;
  stats.lastUpdateMs = millis();
  lastDataMs         = millis();

  nearest = best;
  if (nearest.valid && nearest.distanceKm < stats.closestEver)
    stats.closestEver = nearest.distanceKm;

  Serial.printf("[fetch] inView=%u blips=%u rotor=%u nearest=%s cat=%d dist=%.1fkm valid=%d heap=%u\n",
                count, blipCount, rotorSeen, nearest.valid ? nearest.callsign : "-",
                nearest.valid ? nearest.category : -1,
                nearest.valid ? nearest.distanceKm : 0.0, nearest.valid,
                ESP.getFreeHeap());
  return true;
}

// ---------------------------------------------------------------------------
// Weather (Open-Meteo, no API key required)
// ---------------------------------------------------------------------------
bool fetchWeather() {
  WiFiClientSecure client;
  client.setInsecure();
  client.setBufferSizes(16384, 512);

  HTTPClient https;
  https.setReuse(false);
  String url = "https://api.open-meteo.com/v1/forecast?latitude=";
  url += String(HOME_LAT, 4);
  url += "&longitude=" + String(HOME_LON, 4);
  url += "&current=temperature_2m,relative_humidity_2m,weather_code,wind_speed_10m";
  url += "&hourly=temperature_2m,weather_code&forecast_hours=8&timezone=auto";
  if (!https.begin(client, url)) return false;

  int code = https.GET();
  Serial.printf("[wx] HTTP %d\n", code);
  if (code != HTTP_CODE_OK) { https.end(); return false; }
  String payload = https.getString();
  https.end();

  JsonDocument filter;
  filter["current"] = true;
  filter["hourly"]  = true;
  JsonDocument doc;
  if (deserializeJson(doc, payload, DeserializationOption::Filter(filter))) return false;
  JsonObject c = doc["current"];
  if (c.isNull()) return false;

  weather.tempC    = c["temperature_2m"]       | 0.0f;
  weather.humidity = c["relative_humidity_2m"] | 0;
  weather.code     = c["weather_code"]         | 0;
  weather.windKmh  = c["wind_speed_10m"]       | 0.0f;
  weather.valid    = true;

  // short forecast: a few hours ahead (index 0 of forecast_hours == now)
  JsonArray ht = doc["hourly"]["temperature_2m"].as<JsonArray>();
  JsonArray hc = doc["hourly"]["weather_code"].as<JsonArray>();
  int nowH = -1;
  if (timeReady()) {
    time_t t = time(nullptr);
    struct tm lt;
    localtime_r(&t, &lt);
    nowH = lt.tm_hour;
  }
  const int offs[FC_N] = {2, 4, 6};
  fcCount = 0;
  for (uint8_t k = 0; k < FC_N; k++) {
    int idx = offs[k];
    if ((int)ht.size() > idx) {
      fcast[fcCount].tempC = ht[idx] | 0.0f;
      fcast[fcCount].code  = hc[idx] | 0;
      fcast[fcCount].hour  = (nowH < 0) ? -1 : ((nowH + idx) % 24);
      fcCount++;
    }
  }

  Serial.printf("[wx] %.1fC hum=%d%% wind=%.0f code=%d fc=%u\n",
                weather.tempC, weather.humidity, weather.windKmh, weather.code, fcCount);
  return true;
}

// Short label and icon-kind (0 sun,1 part,2 cloud,3 fog,4 rain,5 snow,6 storm)
// for a WMO weather code.
const char* wxText(int code) {
  if (code == 0)                  return "CLEAR";
  if (code <= 2)                  return "PARTLY";
  if (code == 3)                  return "OVERCAST";
  if (code == 45 || code == 48)   return "FOG";
  if (code >= 51 && code <= 57)   return "DRIZZLE";
  if (code >= 61 && code <= 67)   return "RAIN";
  if (code >= 71 && code <= 77)   return "SNOW";
  if (code >= 80 && code <= 82)   return "SHOWERS";
  if (code >= 85 && code <= 86)   return "SNOW";
  if (code >= 95)                 return "STORM";
  return "WX";
}
int wxKind(int code) {
  if (code == 0)                                       return 0;
  if (code <= 2)                                       return 1;
  if (code == 3)                                       return 2;
  if (code == 45 || code == 48)                        return 3;
  if ((code >= 51 && code <= 67) || (code >= 80 && code <= 82)) return 4;
  if ((code >= 71 && code <= 77) || (code >= 85 && code <= 86)) return 5;
  if (code >= 95)                                      return 6;
  return 2;
}

// ---------------------------------------------------------------------------
// Route / airline / ETA (hexdb.io, free, no key)
// ---------------------------------------------------------------------------
// Generic HTTPS GET into a String. Returns true on HTTP 200.
bool httpGetString(const String& url, String& out) {
  WiFiClientSecure client;
  client.setInsecure();
  client.setBufferSizes(16384, 512);
  HTTPClient https;
  https.setReuse(false);
  if (!https.begin(client, url)) return false;
  int code = https.GET();
  if (code != HTTP_CODE_OK) { https.end(); return false; }
  out = https.getString();
  https.end();
  return true;
}

// Offline ICAO airline-designator table (first 3 letters of the callsign).
struct Airline { const char* code; const char* name; };
const Airline AIRLINES[] = {
  {"RYR","Ryanair"},   {"EJU","easyJet EU"},{"EZY","easyJet"},   {"WZZ","Wizz Air"},
  {"VOE","Volotea"},   {"VLG","Vueling"},   {"ITY","ITA Airways"},{"AZA","Alitalia"},
  {"AFR","Air France"},{"DLH","Lufthansa"}, {"BAW","British AW"},{"KLM","KLM"},
  {"IBE","Iberia"},    {"SWR","SWISS"},     {"AUA","Austrian"},  {"TAP","TAP Air"},
  {"SAS","SAS"},       {"FIN","Finnair"},   {"LOT","LOT Polish"},{"THY","Turkish"},
  {"UAE","Emirates"},  {"QTR","Qatar"},     {"ETD","Etihad"},    {"ELY","El Al"},
  {"AEE","Aegean"},    {"TRA","Transavia"}, {"NAX","Norwegian"}, {"EWG","Eurowings"},
  {"BEL","Brussels"},  {"TVF","Transavia"}, {"ENT","Enter Air"}, {"DAL","Delta"},
  {"UAL","United"},    {"AAL","American"},  {"ACA","Air Canada"},{"UPS","UPS"},
  {"FDX","FedEx"},     {"BCS","DHL Air"},   {"MSR","EgyptAir"},  {"RAM","Royal Air Maroc"},
  {"AZU","Azul"},      {"QFA","Qantas"},    {"SIA","Singapore"}, {"NJE","NetJets"},
  {"EXS","Jet2"},      {"WUK","Wizz UK"},   {"NSZ","Norse"},     {"MMZ","euroAtlantic"},
};

const char* airlineName(const char* callsign) {
  static char fb[4];
  if (!callsign || strlen(callsign) < 3) return "GA / Private";
  char p[4] = { (char)toupper(callsign[0]), (char)toupper(callsign[1]),
                (char)toupper(callsign[2]), 0 };
  for (auto& a : AIRLINES) if (strcmp(a.code, p) == 0) return a.name;
  strcpy(fb, p);
  return fb;   // unknown -> show the 3-letter operator code
}

// Look up departure/arrival airports (and arrival coords for ETA) for a
// callsign. Always fills the airline; route/ETA are best-effort.
void fetchRoute(const char* callsign) {
  strncpy(routeInfo.airline, airlineName(callsign), sizeof(routeInfo.airline) - 1);
  routeInfo.airline[sizeof(routeInfo.airline) - 1] = '\0';
  routeInfo.haveRoute = false;
  routeInfo.haveArrPos = false;
  routeInfo.dep[0] = routeInfo.arr[0] = '\0';
  strncpy(routeInfo.callsign, callsign, sizeof(routeInfo.callsign) - 1);
  routeInfo.callsign[sizeof(routeInfo.callsign) - 1] = '\0';

  String payload;
  if (httpGetString(String("https://hexdb.io/api/v1/route/icao/") + callsign, payload)) {
    JsonDocument d;
    if (!deserializeJson(d, payload)) {
      const char* r = d["route"] | "";
      const char* dash = strchr(r, '-');
      if (r[0] && dash) {
        size_t dl = dash - r;
        if (dl < sizeof(routeInfo.dep)) {
          strncpy(routeInfo.dep, r, dl);
          routeInfo.dep[dl] = '\0';
          strncpy(routeInfo.arr, dash + 1, sizeof(routeInfo.arr) - 1);
          routeInfo.arr[sizeof(routeInfo.arr) - 1] = '\0';
          routeInfo.haveRoute = true;
        }
      }
    }
  }

  if (routeInfo.haveRoute && routeInfo.arr[0]) {
    String ap;
    if (httpGetString(String("https://hexdb.io/api/v1/airport/icao/") + routeInfo.arr, ap)) {
      JsonDocument d;
      if (!deserializeJson(d, ap) && !d["latitude"].isNull()) {
        routeInfo.arrLat = d["latitude"]  | 0.0;
        routeInfo.arrLon = d["longitude"] | 0.0;
        routeInfo.haveArrPos = true;
      }
    }
  }

  Serial.printf("[route] %s %s %s>%s eta=%s\n", callsign, routeInfo.airline,
                routeInfo.haveRoute ? routeInfo.dep : "?",
                routeInfo.haveRoute ? routeInfo.arr : "?",
                routeInfo.haveArrPos ? "yes" : "no");
}

// ---------------------------------------------------------------------------
// Time
// ---------------------------------------------------------------------------
bool timeReady() { return time(nullptr) > 1700000000; }

void fmtClock(char* buf, size_t n, bool withSecs) {
  if (!timeReady()) { strncpy(buf, withSecs ? "--:--:--" : "--:--", n); return; }
  time_t t = time(nullptr);
  struct tm lt;
  localtime_r(&t, &lt);
  strftime(buf, n, withSecs ? "%H:%M:%S" : "%H:%M", &lt);
}

// Forward great-circle position: move (lat,lon) by distM metres along trackDeg.
// ---------------------------------------------------------------------------
// Piezo buzzer (rotorcraft alerts)
//
// Everything here is non-blocking: the render loop runs at ~30 fps and any
// delay() would visibly stutter the radar sweep. tone() on the ESP8266 is
// timer-driven and returns immediately, so a multi-chirp pattern is sequenced
// by scheduling the next chirp rather than sleeping between them.
// ---------------------------------------------------------------------------
#if BUZZER_ENABLE

// Level that leaves the buzzer silent. See BUZZER_ACTIVE_LOW in config.h.
#define BUZZER_IDLE_LEVEL (BUZZER_ACTIVE_LOW ? HIGH : LOW)

static uint8_t  chirpsLeft  = 0;
static bool     chirpOn     = false;   // a chirp is currently sounding
static uint16_t chirpFreq   = 0;
static uint16_t chirpOnMs   = 0;
static uint16_t chirpGapMs  = 0;
static uint32_t chirpNextMs = 0;

// Suppress alerts overnight. Falls open (audible) until NTP has synced, so a
// clock that never sets cannot silence the buzzer forever.
bool buzzerQuietNow() {
  if (BUZZER_QUIET_START == BUZZER_QUIET_END) return false;   // disabled
  if (!timeReady()) return false;
  time_t     t  = time(nullptr);
  struct tm* lt = localtime(&t);
  int h = lt->tm_hour;
  if (BUZZER_QUIET_START < BUZZER_QUIET_END)
    return h >= BUZZER_QUIET_START && h < BUZZER_QUIET_END;
  return h >= BUZZER_QUIET_START || h < BUZZER_QUIET_END;     // wraps midnight
}

// Queue a pattern. A pattern already in flight wins, so a sweep blip cannot
// stomp on the tail of a loiter alert.
void buzzerChirp(uint8_t count, uint16_t freq, uint16_t onMs, uint16_t gapMs) {
  if (chirpsLeft > 0 || chirpOn) return;
  if (buzzerQuietNow()) return;
  chirpsLeft  = count;
  chirpFreq   = freq;
  chirpOnMs   = onMs;
  chirpGapMs  = gapMs;
  chirpNextMs = millis();
}

// Advance the chirp pattern. Called every loop().
//
// This drives tone()/noTone() itself rather than using tone()'s duration
// argument, because the core's noTone() ends with digitalWrite(pin, 0) and an
// active-low module reads that as "sound forever". Stopping by hand is the
// only way to put the pin back to its true idle level afterwards.
void buzzerService() {
  if (chirpsLeft == 0 && !chirpOn) return;
  uint32_t now = millis();
  if ((int32_t)(now - chirpNextMs) < 0) return;

  if (chirpOn) {
    noTone(PIN_BUZZER);
    digitalWrite(PIN_BUZZER, BUZZER_IDLE_LEVEL);
    chirpOn     = false;
    chirpNextMs = now + chirpGapMs;
  } else {
    tone(PIN_BUZZER, chirpFreq);
    chirpOn     = true;
    chirpsLeft--;
    chirpNextMs = now + chirpOnMs;
  }
}

// The three voices, deliberately distinguishable without looking at the screen.
// All sit inside the 2-5 kHz band where these piezo elements are loudest;
// below ~2 kHz they go noticeably quiet.
void buzzerSweepBlip()  { buzzerChirp(1, 4000,  25,  40); }  // crisp tick
void buzzerAcquire()    { buzzerChirp(2, 3000,  60,  70); }  // two-tone
void buzzerLoiter()     { buzzerChirp(3, 2200, 120, 100); }  // lowest, insistent

#else
inline bool buzzerQuietNow() { return true; }
inline void buzzerChirp(uint8_t, uint16_t, uint16_t, uint16_t) {}
inline void buzzerService()  {}
inline void buzzerSweepBlip(){}
inline void buzzerAcquire()  {}
inline void buzzerLoiter()   {}
#endif

// Sweep angle on the previous radar frame, so the sweep-crossing blip can tell
// which bearings the beam passed over since last time.
float prevSweepDeg = 0.0f;

// True if the sweep crossed `target` between the previous frame and this one.
// The 30 deg ceiling rejects the large jump seen on the first frame after the
// radar screen comes back around, which would otherwise fire a stray blip.
bool sweptPast(float prev, float cur, float target) {
  float travelled = fmodf(cur - prev + 360.0f, 360.0f);
  if (travelled <= 0.0f || travelled > 30.0f) return false;
  float offset = fmodf(target - prev + 360.0f, 360.0f);
  return offset <= travelled;
}

void projectLatLon(double lat, double lon, float trackDeg, double distM,
                   double& outLat, double& outLon) {
  double dr = distM / 6371000.0;
  double b  = deg2rad(trackDeg);
  double la = deg2rad(lat), lo = deg2rad(lon);
  double nla = asin(sin(la) * cos(dr) + cos(la) * sin(dr) * cos(b));
  double nlo = lo + atan2(sin(b) * sin(dr) * cos(la), cos(dr) - sin(la) * sin(nla));
  outLat = rad2deg(nla);
  outLon = rad2deg(nlo);
}

// ---------------------------------------------------------------------------
// WEAPONS SYSTEM page
//
// A themed air-defense reference display layered over the ADS-B data already
// being fetched. It classifies the current nearest contact, picks a matching
// real-world US air-defense system from a small flash-resident table, and
// shows a track-lead angle plus notional envelope figures.
//
// Scope, deliberately: every contact is FRIENDLY and firing authorization is
// always denied. Nothing here is connected to anything, no real fire-control
// or doctrinal engagement logic is implemented, and PK is a display heuristic
// (see calcPk) rather than a lethality estimate -- there is no public data
// from which a real one could be derived, so it is labelled NOTIONAL.
// ---------------------------------------------------------------------------

static const char WS_A_DES[] PROGMEM = "MIM-104";
static const char WS_A_NAM[] PROGMEM = "PATRIOT";
static const char WS_A_BRN[] PROGMEM = "US ARMY";
static const char WS_A_ROL[] PROGMEM = "AREA DEFENSE";
static const char WS_B_DES[] PROGMEM = "AN/TWQ-1";
static const char WS_B_NAM[] PROGMEM = "AVENGER";
static const char WS_B_BRN[] PROGMEM = "US ARMY";
static const char WS_B_ROL[] PROGMEM = "SHORT RANGE";
static const char WS_C_DES[] PROGMEM = "M-SHORAD";
static const char WS_C_NAM[] PROGMEM = "STRYKER SHORAD";
static const char WS_C_BRN[] PROGMEM = "US ARMY";
static const char WS_C_ROL[] PROGMEM = "POINT DEFENSE";
static const char WS_D_DES[] PROGMEM = "NASAMS";
static const char WS_D_NAM[] PROGMEM = "NASAMS";
static const char WS_D_BRN[] PROGMEM = "US / NORWAY";
static const char WS_D_ROL[] PROGMEM = "AREA DEFENSE";
static const char WS_E_DES[] PROGMEM = "SM-2";
static const char WS_E_NAM[] PROGMEM = "AEGIS";
static const char WS_E_BRN[] PROGMEM = "US NAVY";
static const char WS_E_ROL[] PROGMEM = "FLEET DEFENSE";
static const char WS_F_DES[] PROGMEM = "MADIS";
static const char WS_F_NAM[] PROGMEM = "MADIS";
static const char WS_F_BRN[] PROGMEM = "USMC EXPD";
static const char WS_F_ROL[] PROGMEM = "POINT DEFENSE";

// Index constants keep selectWeaponSystem() readable.
enum : uint8_t { WS_PATRIOT, WS_AVENGER, WS_SHORAD, WS_NASAMS, WS_AEGIS, WS_MADIS, WS_COUNT };

static const WeaponSystemRecord WEAPON_DB[WS_COUNT] PROGMEM = {
  { WS_A_DES, WS_A_NAM, WS_A_BRN, WS_A_ROL, 160, 3, 78, 1400, 15, (uint8_t)AirframeClass::FIXED_WING },
  { WS_B_DES, WS_B_NAM, WS_B_BRN, WS_B_ROL,   8, 1, 12,  750,  5, (uint8_t)AirframeClass::HELICOPTER },
  { WS_C_DES, WS_C_NAM, WS_C_BRN, WS_C_ROL,   8, 1, 13,  750,  5, (uint8_t)AirframeClass::UAV },
  { WS_D_DES, WS_D_NAM, WS_D_BRN, WS_D_ROL,  30, 2, 45, 1020, 10, (uint8_t)AirframeClass::FIXED_WING },
  { WS_E_DES, WS_E_NAM, WS_E_BRN, WS_E_ROL, 170, 4, 79, 1200, 12, (uint8_t)AirframeClass::FIXED_WING },
  { WS_F_DES, WS_F_NAM, WS_F_BRN, WS_F_ROL,   6, 1,  8,  750,  4, (uint8_t)AirframeClass::HELICOPTER },
};

// Pull one record out of flash into a local copy. The strings it points at stay
// in flash, so read them with copyPgm() rather than dereferencing directly.
void loadWeapon(uint8_t idx, WeaponSystemRecord& out) {
  if (idx >= WS_COUNT) idx = WS_PATRIOT;
  memcpy_P(&out, &WEAPON_DB[idx], sizeof(WeaponSystemRecord));
}

void copyPgm(char* dst, size_t n, const char* pgmStr) {
  strncpy_P(dst, pgmStr, n - 1);
  dst[n - 1] = '\0';
}

// Shortest signed angular difference, -180..+180.
float normalizeSignedAngle(float degrees) {
  while (degrees >  180.0f) degrees -= 360.0f;
  while (degrees < -180.0f) degrees += 360.0f;
  return degrees;
}

double calculateInitialBearingDeg(double lat1, double lon1, double lat2, double lon2) {
  return bearingDeg(lat1, lon1, lat2, lon2);   // same great-circle formula
}

// Project a track forward along its reported ground course. Speed is metres
// per second: OpenSky reports m/s directly, so there is no knots conversion.
void projectTrackPosition(double lat, double lon, float trackDeg,
                          double speedMs, uint16_t seconds,
                          double& outLat, double& outLon) {
  projectLatLon(lat, lon, trackDeg, speedMs * (double)seconds, outLat, outLon);
}

// SOLUTION: how far the contact's bearing *from the device* swings over the
// look-ahead window. Positive = clockwise/right. This is a track-lead angle
// for display, not a firing solution.
bool calculateTrackLeadDeg(double devLat, double devLon,
                           double acLat, double acLon,
                           float trackDeg, double speedMs,
                           uint16_t seconds, float& outDeg) {
  if (!isfinite(acLat) || !isfinite(acLon))       return false;
  if (acLat < -90.0 || acLat > 90.0)              return false;
  if (acLon < -180.0 || acLon > 180.0)            return false;
  if (!isfinite(speedMs) || speedMs < TRACK_MIN_SPEED_MS) return false;
  if (!isfinite(trackDeg))                        return false;

  double pLat, pLon;
  projectTrackPosition(acLat, acLon, trackDeg, speedMs, seconds, pLat, pLon);
  double b0 = calculateInitialBearingDeg(devLat, devLon, acLat, acLon);
  double b1 = calculateInitialBearingDeg(devLat, devLon, pLat, pLon);
  float  d  = normalizeSignedAngle((float)(b1 - b0));
  if (!isfinite(d)) return false;
  outDeg = d;
  return true;
}

AltitudeBand classifyAltitude(float altitudeFt, bool haveAltitude) {
  if (!haveAltitude || !isfinite(altitudeFt)) return AltitudeBand::UNKNOWN;
  if (altitudeFt <= ALT_VLOW_MAX_FT) return AltitudeBand::VERY_LOW;
  if (altitudeFt <= ALT_LOW_MAX_FT)  return AltitudeBand::LOW_ALT;
  if (altitudeFt <= ALT_MED_MAX_FT)  return AltitudeBand::MEDIUM;
  if (altitudeFt <= ALT_HIGH_MAX_FT) return AltitudeBand::MED_HIGH;
  return AltitudeBand::HIGH_ALT;
}

// Built on the emitter category the rest of the firmware already resolves,
// which is real ADS-B data when present and a kinematic guess otherwise.
AirframeClass classifyAirframe(int category) {
  switch (category) {
    case 8:  return AirframeClass::HELICOPTER;
    case 14: return AirframeClass::UAV;
    case 2: case 3: case 4: case 5: case 6: case 7: case 9:
             return AirframeClass::FIXED_WING;
    default: return AirframeClass::UNKNOWN;
  }
}

// "Threat" here means aspect: how directly the contact is tracking over the
// device, tightened by range. Everything is friendly regardless -- this drives
// nothing but the label.
ThreatLevel classifyThreat(double bearingFromDevice, float trackDeg,
                           double distanceKm, float altitudeFt, bool haveAlt,
                           bool valid) {
  if (!valid || !isfinite(trackDeg) || !isfinite(distanceKm))
    return ThreatLevel::UNKNOWN;

  // Bearing the contact would fly to pass over the device.
  double inbound = fmod(bearingFromDevice + 180.0, 360.0);
  float  aspect  = fabsf(normalizeSignedAngle((float)(trackDeg - inbound)));

  // Slant range, not ground distance: altitude is most of how far away an
  // aircraft actually is. Overhead at FL350 is 10.7 km, which is why a
  // high-altitude overflight can no longer reach HIGH on ground track alone.
  double altKm = (haveAlt && isfinite(altitudeFt)) ? altitudeFt * 0.0003048 : 0.0;
  double slant = sqrt(distanceKm * distanceKm + altKm * altKm);

  // No altitude means we cannot show it is low, so HIGH is off the table.
  // HIGH needs low *and* close: the ceiling below is what makes it mean
  // "near enough to read markings" rather than merely "nearly overhead".
  if (haveAlt && altitudeFt <= THREAT_HIGH_MAX_FT &&
      aspect < THREAT_HIGH_ASPECT_DEG && slant < THREAT_HIGH_SLANT_KM)
    return ThreatLevel::HIGH_THREAT;
  if (aspect < THREAT_MED_ASPECT_DEG && slant < THREAT_MED_SLANT_KM)
    return ThreatLevel::MED_THREAT;
  return ThreatLevel::LOW_THREAT;
}

// VERY_LOW and LOW both mean "short-range system territory" for selection.
bool isLowBand(AltitudeBand b) {
  return b == AltitudeBand::VERY_LOW || b == AltitudeBand::LOW_ALT;
}

uint8_t selectWeaponSystem(AirframeClass airframe, AltitudeBand band, double distanceKm) {
#if DEFENSE_THEME == DEFENSE_THEME_NAVY
  return WS_AEGIS;
#elif DEFENSE_THEME == DEFENSE_THEME_MARINE
  // MADIS is a short-range SHORAD / counter-UAS system, so it only owns the
  // bottom of the stack: rotorcraft, UAVs, and traffic genuinely down low.
  // Anything higher steps up the ladder rather than being claimed by it.
  if (airframe == AirframeClass::HELICOPTER || airframe == AirframeClass::UAV ||
      band == AltitudeBand::VERY_LOW)
    return WS_MADIS;
  if (band == AltitudeBand::LOW_ALT || band == AltitudeBand::MEDIUM) return WS_NASAMS;
  return WS_PATRIOT;
#else
  if (airframe == AirframeClass::HELICOPTER) return WS_AVENGER;
  if (airframe == AirframeClass::UAV && isLowBand(band)) return WS_SHORAD;
  if (airframe == AirframeClass::FIXED_WING && isLowBand(band)) {
    // Prefer the area-defense reference once the contact is beyond the
    // short-range system's published reach.
    return (distanceKm > 8.0) ? WS_NASAMS : WS_AVENGER;
  }
  return WS_PATRIOT;
#endif
}

// Is the contact inside the selected system's published envelope? Range and
// ceiling only -- the honest limit of what open figures support.
Envelope classifyEnvelope(const WeaponSystemRecord& w, double distanceKm,
                          float altitudeFt, bool haveAlt, bool valid) {
  if (!valid || !isfinite(distanceKm)) return Envelope::NO_DATA;
  if (distanceKm > w.maxRangeKm)       return Envelope::TOO_FAR;
  if (distanceKm < w.minRangeKm)       return Envelope::TOO_CLOSE;
  if (haveAlt && isfinite(altitudeFt) && altitudeFt > w.ceilingKft * 1000.0f)
    return Envelope::ALT_OUT;
  return Envelope::INSIDE;
}

// Notional time to intercept: straight-line range at the published average
// missile speed, plus the system's reaction time. Ignores lead pursuit,
// boost/coast profile and every other real factor -- it is a plausible-looking
// number for a desk display, not a time-of-flight prediction.
bool calcInterceptSeconds(const WeaponSystemRecord& w, double distanceKm,
                          Envelope env, float& outSec) {
  if (env != Envelope::INSIDE || w.missileSpeedMps == 0) return false;
  float t = (float)(distanceKm * 1000.0) / (float)w.missileSpeedMps + w.reactionS;
  if (!isfinite(t)) return false;
  outSec = t;
  return true;
}

// PK is invented. There is no public dataset that would let anyone compute a
// real probability of kill, so rather than dress a fabricated constant up as
// fact this is an explicit geometric heuristic: best mid-envelope, degraded
// near the edges, at the ceiling, and off-aspect. Displayed as NOTIONAL.
bool calcPk(const WeaponSystemRecord& w, double distanceKm, float altitudeFt,
            bool haveAlt, ThreatLevel threat, Envelope env, float& outPk) {
  if (env != Envelope::INSIDE) return false;

  float span = (float)(w.maxRangeKm - w.minRangeKm);
  if (span <= 0.0f) return false;
  float into = ((float)distanceKm - w.minRangeKm) / span;    // 0 near, 1 far
  float rangeFit = 1.0f - fabsf(into - 0.35f) * 1.4f;        // peak just inside

  float altFit = 1.0f;
  if (haveAlt && isfinite(altitudeFt)) {
    float ceilFt = w.ceilingKft * 1000.0f;
    if (ceilFt > 0.0f) altFit = 1.0f - 0.5f * (altitudeFt / ceilFt);
  }

  float aspectFit = (threat == ThreatLevel::HIGH_THREAT)   ? 1.0f
                  : (threat == ThreatLevel::MED_THREAT) ? 0.85f : 0.7f;

  float pk = rangeFit * altFit * aspectFit;
  if (pk < 0.05f) pk = 0.05f;
  if (pk > 0.95f) pk = 0.95f;
  if (!isfinite(pk)) return false;
  outPk = pk;
  return true;
}

const char* airframeText(AirframeClass a) {
  switch (a) {
    case AirframeClass::FIXED_WING: return "FIXED-WING";
    case AirframeClass::HELICOPTER: return "HELICOPTER";
    case AirframeClass::UAV:        return "UAV";
    default:                        return "UNKNOWN";
  }
}

const char* altBandText(AltitudeBand b) {
  switch (b) {
    case AltitudeBand::VERY_LOW: return "VERY LOW";
    case AltitudeBand::LOW_ALT:  return "LOW";
    case AltitudeBand::MEDIUM:   return "MEDIUM";
    case AltitudeBand::MED_HIGH: return "MED-HIGH";
    case AltitudeBand::HIGH_ALT: return "HIGH";
    default:                     return "UNKNOWN";
  }
}

const char* threatText(ThreatLevel t) {
  switch (t) {
    case ThreatLevel::LOW_THREAT:    return "LOW";
    case ThreatLevel::MED_THREAT: return "MEDIUM";
    case ThreatLevel::HIGH_THREAT:   return "HIGH";
    default:                  return "UNKNOWN";
  }
}

const char* envelopeText(Envelope e) {
  switch (e) {
    case Envelope::INSIDE:    return "IN";
    case Envelope::TOO_FAR:   return "OUT-FAR";
    case Envelope::TOO_CLOSE: return "OUT-MIN";
    case Envelope::ALT_OUT:   return "OUT-ALT";
    default:                  return "---";
  }
}

// Format the SOLUTION field: R/L/C prefix, or "---.-" with no valid track.
void formatSolution(char* buf, size_t n, bool valid, float deg) {
  if (!valid)                snprintf(buf, n, "---.-");
  else if (fabsf(deg) < 0.05f) snprintf(buf, n, "C000.0");
  else snprintf(buf, n, "%c%05.1f", deg > 0.0f ? 'R' : 'L', fabsf(deg));
}

// ---------------------------------------------------------------------------
// Drawing helpers
// ---------------------------------------------------------------------------
// Tactical header: title + screen index on the left, NTP clock on the right.
void drawHeader(const char* title) {
  u8g2.setFont(u8g2_font_5x7_tr);
  char left[20];
  snprintf(left, sizeof(left), "%s %d/%d", title, screen + 1, NUM_SCREENS);
  u8g2.drawStr(2, 6, left);

  char t[10];
  fmtClock(t, sizeof(t), true);
  u8g2.drawStr(127 - u8g2.getStrWidth(t), 6, t);

  u8g2.drawHLine(0, 8, 128);
  u8g2.drawVLine(0, 0, 3);     // HUD corner ticks
  u8g2.drawVLine(127, 0, 3);
}

// Filled cloud silhouette, left edge near (cx-7), vertically around cy.
void drawCloud(int cx, int cy) {
  u8g2.drawDisc(cx - 5, cy, 4);
  u8g2.drawDisc(cx + 1, cy - 3, 5);
  u8g2.drawDisc(cx + 6, cy, 4);
  u8g2.drawBox(cx - 5, cy, 12, 5);
}

void drawWeatherIcon(int cx, int cy, int kind) {
  switch (kind) {
    case 0: { // sun
      u8g2.drawDisc(cx, cy, 5);
      for (int a = 0; a < 360; a += 45) {
        double r = deg2rad(a);
        u8g2.drawLine(cx + (int)(cos(r) * 7), cy + (int)(sin(r) * 7),
                      cx + (int)(cos(r) * 9), cy + (int)(sin(r) * 9));
      }
      break;
    }
    case 1: // partly cloudy
      u8g2.drawDisc(cx - 3, cy - 4, 4);
      drawCloud(cx + 2, cy + 2);
      break;
    case 3: // fog
      drawCloud(cx, cy - 2);
      for (int i = 0; i < 3; i++) u8g2.drawHLine(cx - 7, cy + 5 + i * 2, 15);
      break;
    case 4: // rain
      drawCloud(cx, cy - 2);
      for (int i = -4; i <= 6; i += 5) u8g2.drawLine(cx + i, cy + 4, cx + i - 2, cy + 8);
      break;
    case 5: // snow
      drawCloud(cx, cy - 2);
      for (int i = -4; i <= 6; i += 5) {
        u8g2.drawPixel(cx + i, cy + 6);
        u8g2.drawHLine(cx + i - 1, cy + 6, 3);
        u8g2.drawVLine(cx + i, cy + 5, 3);
      }
      break;
    case 6: // storm
      drawCloud(cx, cy - 2);
      u8g2.drawLine(cx, cy + 4, cx - 3, cy + 7);
      u8g2.drawLine(cx - 3, cy + 7, cx + 1, cy + 7);
      u8g2.drawLine(cx + 1, cy + 7, cx - 2, cy + 10);
      break;
    default: // overcast / generic cloud
      drawCloud(cx, cy);
      break;
  }
}

// Arrow pointing toward `angle` (0 = up/north), centred at (cx,cy).
void drawArrow(int cx, int cy, int r, double angleDeg) {
  double a = deg2rad(angleDeg);
  // tip
  int tx = cx + (int)(sin(a) * r);
  int ty = cy - (int)(cos(a) * r);
  // tail
  int bx = cx - (int)(sin(a) * r);
  int by = cy + (int)(cos(a) * r);
  u8g2.drawLine(bx, by, tx, ty);
  // arrow head
  double left  = a + deg2rad(150);
  double right = a - deg2rad(150);
  u8g2.drawLine(tx, ty, tx + (int)(sin(left)  * (r / 2)), ty - (int)(cos(left)  * (r / 2)));
  u8g2.drawLine(tx, ty, tx + (int)(sin(right) * (r / 2)), ty - (int)(cos(right) * (r / 2)));
}

// Short label for an OpenSky emitter category.
const char* typeName(int cat) {
  switch (cat) {
    case 2:  return "Light";
    case 3:  return "Small";
    case 4:  return "Airliner";
    case 5:  return "Heavy";
    case 6:  return "Heavy";
    case 7:  return "Jet";
    case 8:  return "Heli";
    case 9:  return "Glider";
    case 10: return "Balloon";
    case 14: return "Drone";
    default: return "Aircraft";
  }
}

// Icon (~16x12) for an aircraft type, centred at (cx,cy).
void drawTypeIcon(int cx, int cy, int cat) {
  switch (cat) {
    case 8:  // helicopter
      u8g2.drawDisc(cx - 1, cy, 2);
      u8g2.drawHLine(cx - 7, cy - 3, 15);          // main rotor
      u8g2.drawLine(cx + 1, cy, cx + 7, cy + 1);   // tail boom
      u8g2.drawVLine(cx + 7, cy - 2, 5);           // tail rotor
      break;
    case 9:  // glider (long slim wings)
      u8g2.drawHLine(cx - 8, cy, 17);
      u8g2.drawVLine(cx, cy - 2, 7);
      u8g2.drawHLine(cx - 2, cy + 5, 5);
      break;
    case 10: // balloon / lighter-than-air
      u8g2.drawCircle(cx, cy - 2, 4);
      u8g2.drawLine(cx - 3, cy + 1, cx - 1, cy + 5);
      u8g2.drawLine(cx + 3, cy + 1, cx + 1, cy + 5);
      u8g2.drawFrame(cx - 1, cy + 5, 3, 2);
      break;
    case 14: // drone (quadcopter)
      u8g2.drawBox(cx - 1, cy - 1, 3, 3);
      u8g2.drawLine(cx - 5, cy - 4, cx + 5, cy + 4);
      u8g2.drawLine(cx + 5, cy - 4, cx - 5, cy + 4);
      u8g2.drawCircle(cx - 5, cy - 4, 2);
      u8g2.drawCircle(cx + 5, cy - 4, 2);
      u8g2.drawCircle(cx - 5, cy + 4, 2);
      u8g2.drawCircle(cx + 5, cy + 4, 2);
      break;
    case 2:
    case 3:  // light / small plane (straight wings)
      u8g2.drawVLine(cx, cy - 4, 10);
      u8g2.drawHLine(cx - 5, cy - 1, 11);
      u8g2.drawHLine(cx - 2, cy + 4, 5);
      break;
    default: // airliner / generic (swept wings, top view)
      u8g2.drawVLine(cx, cy - 5, 12);
      u8g2.drawTriangle(cx, cy - 1, cx - 7, cy + 3, cx - 1, cy + 1);
      u8g2.drawTriangle(cx, cy - 1, cx + 7, cy + 3, cx + 1, cy + 1);
      u8g2.drawTriangle(cx, cy + 4, cx - 3, cy + 6, cx - 1, cy + 5);
      u8g2.drawTriangle(cx, cy + 4, cx + 3, cy + 6, cx + 1, cy + 5);
      break;
  }
}

// WiFi signal bars (0..4), bottom-aligned at baseline y, growing right.
void drawSignalBars(int x, int y, int rssi) {
  int bars = 0;
  if (rssi >= -55)      bars = 4;
  else if (rssi >= -65) bars = 3;
  else if (rssi >= -75) bars = 2;
  else if (rssi >= -85) bars = 1;
  for (int i = 0; i < 4; i++) {
    int h = 2 + i * 2;
    if (i < bars) u8g2.drawBox(x + i * 3, y - h, 2, h);
    else          u8g2.drawFrame(x + i * 3, y - h, 2, h);
  }
}

int effectiveCategory(const Aircraft& a) {
  return categoryFrom(a.category, a.onGround, a.velocityMs, a.altitudeM);
}
bool isEstimatedType(const Aircraft& a) { return a.category == 0 && !a.onGround; }

// Is the current target a rotorcraft, and is it holding station?
bool nearestIsRotor()  { return nearest.valid && isRotor(effectiveCategory(nearest)); }
bool nearestLoitering() { return nearestIsRotor() && isLoitering(nearest.icao24); }

void screenNearest() {
  drawHeader("TARGET");

  if (!nearest.valid) {
    u8g2.setFont(u8g2_font_6x12_tr);
    u8g2.drawStr(0, 30, "NO TARGET");
    u8g2.drawStr(0, 44, "in range.");
    return;
  }

  u8g2.setFont(u8g2_font_7x14B_tr);
  u8g2.drawStr(0, 24, nearest.callsign);

  // aircraft-type icon, between the callsign and the heading arrow
  drawTypeIcon(82, 16, effectiveCategory(nearest));

  u8g2.setFont(u8g2_font_6x12_tr);
  char line[24];
  snprintf(line, sizeof(line), "%.1f km %s", nearest.distanceKm, compass(nearest.bearingDeg));
  u8g2.drawStr(0, 40, line);

  if (nearest.onGround) {
    u8g2.drawStr(0, 54, "on ground");
  } else {
    snprintf(line, sizeof(line), "%.0f m / FL%03.0f",
             nearest.altitudeM, nearest.altitudeM * 3.28084 / 100.0);
    u8g2.drawStr(0, 54, line);
  }

  // heading arrow + speed on the right
  drawArrow(110, 34, 11, nearest.trackDeg);
  snprintf(line, sizeof(line), "%.0f", nearest.velocityMs * 3.6); // km/h
  u8g2.setFont(u8g2_font_4x6_tr);
  u8g2.drawStr(98, 56, line);
  u8g2.drawStr(98, 63, "km/h");

  // altitude gauge on the far-right column (0..FL400)
  const int gT = 14, gB = 50;
  u8g2.drawFrame(125, gT, 3, gB - gT);
  if (!nearest.onGround) {
    float fl = nearest.altitudeM * 3.28084f / 100.0f;
    float fr = fl / 400.0f;
    if (fr > 1) fr = 1;
    if (fr < 0) fr = 0;
    int fh = (int)((gB - gT - 2) * fr);
    u8g2.drawBox(126, gB - 1 - fh, 1, fh);
  }

  // Rotorcraft alert, inverted so it reads as a banner rather than another
  // data row. Sits in the bottom-left, clear of the km/h readout at x=98.
  // A loitering one is the interesting case, so it says so and blinks.
  if (nearestIsRotor()) {
    bool loiter = nearestLoitering();
    if (!loiter || ((millis() / 500) & 1)) {
      const char* msg = loiter ? "ROTOR LOITER" : "ROTORCRAFT";
      int w = strlen(msg) * 5 + 3;
      u8g2.drawBox(0, 56, w, 8);
      u8g2.setDrawColor(0);
      u8g2.setFont(u8g2_font_5x7_tr);
      u8g2.drawStr(2, 63, msg);
      u8g2.setDrawColor(1);
    }
  }
}

void screenDetails() {
  drawHeader("INTEL");

  if (!nearest.valid) {
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(0, 30, "NO INTEL");
    return;
  }

  bool haveRoute = (strcmp(routeInfo.callsign, nearest.callsign) == 0);
  char line[32];

  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(0, 20, nearest.callsign);

  u8g2.setFont(u8g2_font_5x7_tr);
  snprintf(line, sizeof(line), "LINE %s", haveRoute ? routeInfo.airline : airlineName(nearest.callsign));
  u8g2.drawStr(0, 31, line);

  if (haveRoute && routeInfo.haveRoute)
    snprintf(line, sizeof(line), "RTE  %s > %s", routeInfo.dep, routeInfo.arr);
  else
    snprintf(line, sizeof(line), "RTE  unknown");
  u8g2.drawStr(0, 41, line);

  if (haveRoute && routeInfo.haveArrPos && nearest.velocityMs > 20) {
    double dk  = haversineKm(nearest.lat, nearest.lon, routeInfo.arrLat, routeInfo.arrLon);
    int    min = (int)(dk / (nearest.velocityMs * 3.6) * 60.0);
    snprintf(line, sizeof(line), "ETA  %dh%02dm  %.0fkm", min / 60, min % 60, dk);
  } else {
    snprintf(line, sizeof(line), "ETA  --");
  }
  u8g2.drawStr(0, 51, line);

  snprintf(line, sizeof(line), "ID %s HDG %03.0f", nearest.icao24, nearest.trackDeg);
  u8g2.drawStr(0, 61, line);
}

// North-up radar (PPI). Home at the centre, range rings (outer = 30 km), a
// rotating sweep, and a blip per aircraft. Blips are dead-reckoned from their
// last track+speed so they creep in real time between data refreshes, and use
// radar persistence: bright just after the sweep passes, then a faint dot.
void screenRadar() {
  drawHeader("RADAR");

  const int cx = 31, cy = 37, R = 23;
  // Outer ring range. Match this to the search box (SEARCH_RADIUS_DEG): the box
  // corner is ~sqrt(2)*radius*111 km, so a 0.2 deg box maxes out near 28 km.
  const float MAX_KM = 30.0f;
  float elapsed = (millis() - lastDataMs) / 1000.0f;   // s since last fetch

  // rings + axes
  u8g2.drawCircle(cx, cy, R);
  u8g2.drawCircle(cx, cy, (R * 2) / 3);
  u8g2.drawCircle(cx, cy, R / 3);
  u8g2.drawHLine(cx - R, cy, 2 * R + 1);
  u8g2.drawVLine(cx, cy - R, 2 * R + 1);
  u8g2.drawDisc(cx, cy, 1);

  u8g2.setFont(u8g2_font_4x6_tr);
  u8g2.drawStr(cx - 1, cy - R - 1, "N");

  // wall-facing tick just outside the ring
  {
    double a = deg2rad((double)WALL_HEADING_DEG);
    u8g2.drawDisc(cx + (int)(sin(a) * (R + 2)), cy - (int)(cos(a) * (R + 2)), 1);
  }

  // rotating sweep (~4 s/turn, clockwise)
  float sweepDeg = fmodf(millis() / 12.0f, 360.0f);
  double sw = deg2rad(sweepDeg);
  u8g2.drawLine(cx, cy, cx + (int)(sin(sw) * R), cy - (int)(cos(sw) * R));

  // blips, dead-reckoned + persistence
  int   nearIdx = -1;
  float nearD   = 1e9;
  for (uint8_t i = 0; i < blipCount; i++) {
    double la = blips[i].lat, lo = blips[i].lon;
    if (blips[i].speedMs > 0 && elapsed > 0)
      projectLatLon(blips[i].lat, blips[i].lon, blips[i].track,
                    blips[i].speedMs * elapsed, la, lo);
    double dist = haversineKm(HOME_LAT, HOME_LON, la, lo);
    if (dist < nearD) { nearD = dist; nearIdx = i; }

    float fr = (float)(dist / MAX_KM);
    if (fr > 1) continue;
    int rr  = (int)(fr * R);
    double brg = bearingDeg(HOME_LAT, HOME_LON, la, lo);
    int bx = cx + (int)(sin(deg2rad(brg)) * rr);
    int by = cy - (int)(cos(deg2rad(brg)) * rr);

    if (isRotor(blips[i].cat)) {
      // A cross reads as distinct from the plain dots even at this scale, and
      // rotorcraft deliberately skip the persistence fade -- a special contact
      // should not thin out to one pixel between sweeps. Loitering adds a
      // pulsing ring, which is the part that actually catches the eye.
      u8g2.drawHLine(bx - 2, by, 5);
      u8g2.drawVLine(bx, by - 2, 5);
      if (blips[i].loiter && ((millis() / 400) & 1)) u8g2.drawCircle(bx, by, 4);
#if BUZZER_ENABLE && BUZZER_SWEEP_BLIP
      // Chirp as the beam crosses it -- the classic radar tick, but only for
      // rotorcraft, only inside BUZZER_RANGE_KM, and only while this screen is
      // up, which keeps it to a few ticks per screen cycle instead of a sonar.
      if (dist <= BUZZER_RANGE_KM && sweptPast(prevSweepDeg, sweepDeg, (float)brg))
        buzzerSweepBlip();
#endif
    } else {
      float behind = fmodf(sweepDeg - (float)brg + 360.0f, 360.0f);
      if (behind < 50) u8g2.drawDisc(bx, by, 1);   // freshly swept
      else             u8g2.drawPixel(bx, by);     // fading
    }
  }

  prevSweepDeg = sweepDeg;

  // highlight the closest live contact
  if (nearIdx >= 0) {
    double la = blips[nearIdx].lat, lo = blips[nearIdx].lon;
    if (blips[nearIdx].speedMs > 0 && elapsed > 0)
      projectLatLon(blips[nearIdx].lat, blips[nearIdx].lon, blips[nearIdx].track,
                    blips[nearIdx].speedMs * elapsed, la, lo);
    double dist = haversineKm(HOME_LAT, HOME_LON, la, lo);
    double brg  = bearingDeg(HOME_LAT, HOME_LON, la, lo);
    float fr = (float)(dist / MAX_KM);
    if (fr <= 1) {
      int rr = (int)(fr * R);
      int bx = cx + (int)(sin(deg2rad(brg)) * rr);
      int by = cy - (int)(cos(deg2rad(brg)) * rr);
      u8g2.drawCircle(bx, by, 3);
      u8g2.drawDisc(bx, by, 1);
    }
  }

  // side info panel
  const int px = 62;
  if (!nearest.valid) {
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(px, 32, "NO CONTACT");
    return;
  }

  int ec = effectiveCategory(nearest);
  drawTypeIcon(px + 7, 19, ec);
  u8g2.setFont(u8g2_font_5x7_tr);
  char tname[12];
  snprintf(tname, sizeof(tname), "%s%s", isEstimatedType(nearest) ? "~" : "", typeName(ec));
  u8g2.drawStr(px + 18, 20, tname);
  u8g2.drawStr(px, 32, nearest.callsign);

  char l[20];
  snprintf(l, sizeof(l), "RNG %.0fkm", nearest.distanceKm);
  u8g2.drawStr(px, 43, l);
  snprintf(l, sizeof(l), "BRG %03.0f %s", nearest.bearingDeg, compass(nearest.bearingDeg));
  u8g2.drawStr(px, 54, l);

  u8g2.setFont(u8g2_font_4x6_tr);
  snprintf(l, sizeof(l), "%u CONTACTS", blipCount);
  u8g2.drawStr(px, 63, l);
}

void screenWeather() {
  drawHeader("WX");

  if (!weather.valid) {
    u8g2.setFont(u8g2_font_6x12_tr);
    u8g2.drawStr(18, 38, "no wx data");
    return;
  }

  drawWeatherIcon(18, 30, wxKind(weather.code));

  // big temperature, with a real degree glyph aligned to the baseline
  char t[8];
  snprintf(t, sizeof(t), "%.0f", weather.tempC);
  u8g2.setFont(u8g2_font_logisoso16_tn);
  u8g2.drawStr(40, 34, t);
  int w = u8g2.getStrWidth(t);
  u8g2.setFont(u8g2_font_9x15_tf);
  u8g2.drawStr(40 + w + 2, 34, "\xB0" "C");

  // condition + current humidity/wind
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.drawStr(40, 45, wxText(weather.code));
  u8g2.setFont(u8g2_font_4x6_tr);
  char l[26];
  snprintf(l, sizeof(l), "HUM %d%%  WIND %.0fkm/h", weather.humidity, weather.windKmh);
  u8g2.drawStr(2, 53, l);

  // minimal next-hours forecast strip
  u8g2.drawHLine(0, 55, 128);
  int fx = 2;
  for (uint8_t k = 0; k < fcCount; k++) {
    char fb[12];
    if (fcast[k].hour >= 0) snprintf(fb, sizeof(fb), "%02dh %.0fc", fcast[k].hour, fcast[k].tempC);
    else                    snprintf(fb, sizeof(fb), "+%dh %.0fc", (k + 1) * 2, fcast[k].tempC);
    u8g2.drawStr(fx, 63, fb);
    fx += 44;
  }
}

void screenSystem() {
  drawHeader("SYSTEM");

  // big clock HH:MM:SS
  char clk[12];
  fmtClock(clk, sizeof(clk), true);
  u8g2.setFont(u8g2_font_logisoso16_tn);
  int w = u8g2.getStrWidth(clk);
  int x0 = 4;
  u8g2.drawStr(x0, 30, clk);

  // fast-updating milliseconds
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  char msb[6];
  snprintf(msb, sizeof(msb), ".%03d", (int)(tv.tv_usec / 1000));
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(x0 + w + 2, 30, msb);

  // date
  u8g2.setFont(u8g2_font_5x7_tr);
  char dl[24];
  if (timeReady()) {
    time_t tt = time(nullptr);
    struct tm lt;
    localtime_r(&tt, &lt);
    strftime(dl, sizeof(dl), "%a %Y-%m-%d", &lt);
  } else {
    strcpy(dl, "SYNCING NTP...");
  }
  u8g2.drawStr(2, 42, dl);

  // uptime + target count
  uint32_t up = millis() / 1000;
  char l[30];
  snprintf(l, sizeof(l), "UP %02lu:%02lu:%02lu   TGT %u",
           up / 3600, (up % 3600) / 60, up % 60, stats.inView);
  u8g2.drawStr(2, 52, l);

  // link status: signal bars + details
  drawSignalBars(2, 62, WiFi.RSSI());
  u8g2.setFont(u8g2_font_4x6_tr);
  snprintf(l, sizeof(l), "%ddBm RAM%dk REQ%lu/%lu",
           WiFi.RSSI(), ESP.getFreeHeap() / 1024, stats.requestsOk, stats.requestsFail);
  u8g2.drawStr(18, 62, l);
}

// WEAPONS SYSTEM. One static page, no animation. Everything is drawn in the
// 4x6 font (32 chars across) so nine rows fit under the header without wrap.
void screenWeapons() {
  drawHeader("WEAPONS");

  // Stale data must not carry the previous contact's solution forward.
  bool fresh = nearest.valid && lastDataMs != 0 &&
               (uint32_t)(millis() - lastDataMs) < TRACK_STALE_MS;

  int   cat     = fresh ? effectiveCategory(nearest) : 0;
  AirframeClass af = fresh ? classifyAirframe(cat) : AirframeClass::UNKNOWN;

  bool  haveAlt = fresh && !nearest.onGround && isfinite(nearest.altitudeM);
  float altFt   = haveAlt ? nearest.altitudeM * 3.28084f : NAN;
  AltitudeBand band = classifyAltitude(altFt, haveAlt);

  ThreatLevel threat = classifyThreat(nearest.bearingDeg, nearest.trackDeg,
                                      nearest.distanceKm, altFt, haveAlt, fresh);

  uint8_t wsIdx = selectWeaponSystem(af, band, fresh ? nearest.distanceKm : 0.0);
  WeaponSystemRecord w;
  loadWeapon(wsIdx, w);

  Envelope env = classifyEnvelope(w, nearest.distanceKm, altFt, haveAlt, fresh);

  float solDeg = 0.0f;
  bool  solOk  = fresh && calculateTrackLeadDeg(HOME_LAT, HOME_LON,
                                                nearest.lat, nearest.lon,
                                                nearest.trackDeg, nearest.velocityMs,
                                                TRACK_LOOKAHEAD_SECONDS, solDeg);

  float tof = 0.0f, pk = 0.0f;
  bool  tofOk = calcInterceptSeconds(w, nearest.distanceKm, env, tof);
  bool  pkOk  = calcPk(w, nearest.distanceKm, altFt, haveAlt, threat, env, pk);

  char line[36], des[12], nam[16], brn[14], rol[16], sol[10];
  copyPgm(des, sizeof(des), w.designation);
  copyPgm(nam, sizeof(nam), w.name);
  copyPgm(brn, sizeof(brn), w.branch);
  copyPgm(rol, sizeof(rol), w.role);
  formatSolution(sol, sizeof(sol), solOk, solDeg);

  u8g2.setFont(u8g2_font_4x6_tr);

  snprintf(line, sizeof(line), "AIRFRAME: %s", airframeText(af));
  u8g2.drawStr(0, 15, line);
  snprintf(line, sizeof(line), "ALT BAND: %s", altBandText(band));
  u8g2.drawStr(0, 21, line);

  // Threat left, firing authorization right. Authorization is unconditionally
  // denied: every contact here is friendly and nothing is armed.
  snprintf(line, sizeof(line), "THREAT: %s", threatText(threat));
  u8g2.drawStr(0, 27, line);
  u8g2.drawStr(128 - u8g2.getStrWidth("AUTH:HOLD"), 27, "AUTH:HOLD");

  snprintf(line, sizeof(line), "SOLUTION: %s", sol);
  u8g2.drawStr(0, 33, line);

  if (fresh)
    snprintf(line, sizeof(line), "ENV %s  RNG %03d/%03dKM",
             envelopeText(env), (int)(nearest.distanceKm + 0.5), w.maxRangeKm);
  else
    snprintf(line, sizeof(line), "ENV ---  RNG ---/%03dKM", w.maxRangeKm);
  u8g2.drawStr(0, 39, line);

  if (tofOk && pkOk)
    snprintf(line, sizeof(line), "TOF %03ds  PK %.2f NOTNL", (int)(tof + 0.5f), pk);
  else if (tofOk)
    snprintf(line, sizeof(line), "TOF %03ds  PK ---", (int)(tof + 0.5f));
  else
    snprintf(line, sizeof(line), "TOF ---   PK ---");
  u8g2.drawStr(0, 45, line);

  u8g2.drawHLine(0, 48, 128);

  snprintf(line, sizeof(line), "MATCH: %s", des);
  u8g2.drawStr(0, 57, line);
  u8g2.drawStr(128 - u8g2.getStrWidth(rol), 57, rol);

  snprintf(line, sizeof(line), "%s / %s", nam, brn);
  u8g2.drawStr(0, 63, line);
}

void render() {
  u8g2.clearBuffer();
  switch (screen) {
    case SCR_RADAR:   screenRadar();   break;
    case SCR_TARGET:  screenNearest(); break;
    case SCR_INTEL:   screenDetails(); break;
    case SCR_WEAPONS: screenWeapons(); break;
    case SCR_WX:      screenWeather(); break;
    case SCR_SYSTEM:  screenSystem();  break;
  }
  u8g2.sendBuffer();
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
#if DISPLAY_I2C
// Probe the bus for the panel and point U8g2 at whichever address answers.
// Returns the 7-bit address found, or 0 if nothing ACKed.
static uint8_t detectOledAddress() {
  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  const uint8_t candidates[2] = { OLED_I2C_ADDR, OLED_I2C_ADDR_ALT };
  for (uint8_t i = 0; i < 2; i++) {
    Wire.beginTransmission(candidates[i]);
    if (Wire.endTransmission() == 0) return candidates[i];
  }
  return 0;
}
#endif

void setup() {
  Serial.begin(115200);
  Serial.println();

#if DISPLAY_I2C
  uint8_t addr = detectOledAddress();
  if (addr) {
    Serial.printf("[oled] SSD1306 found at 0x%02X on SDA=GPIO%d SCL=GPIO%d\n",
                  addr, PIN_OLED_SDA, PIN_OLED_SCL);
    u8g2.setI2CAddress(addr << 1);   // U8g2 wants the 8-bit form
  } else {
    Serial.printf("[oled] no I2C device on SDA=GPIO%d SCL=GPIO%d - check "
                  "wiring/power; trying 0x%02X anyway\n",
                  PIN_OLED_SDA, PIN_OLED_SCL, OLED_I2C_ADDR);
  }
#endif

#if BUZZER_ENABLE
  // Park the pin at its idle level before anything else, so the buzzer is not
  // sounding between boot and the first chirp. Which level that is depends on
  // the module: an active-low (PNP) one idles HIGH.
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, BUZZER_IDLE_LEVEL);
  Serial.printf("[buzz] enabled on GPIO%d (%s), range %.0f km, quiet %02d:00-%02d:00\n",
                PIN_BUZZER, BUZZER_ACTIVE_LOW ? "active-low" : "active-high",
                (double)BUZZER_RANGE_KM,
                BUZZER_QUIET_START, BUZZER_QUIET_END);
#endif

  u8g2.begin();
  u8g2.setContrast(180);

  // splash
  u8g2.clearBuffer();
  u8g2.drawFrame(0, 0, 128, 64);
  u8g2.setFont(u8g2_font_7x14B_tr);
  u8g2.drawStr(8, 28, "PLANE SPOTTER");
  u8g2.setFont(u8g2_font_5x7_tr);
  u8g2.drawStr(14, 46, "TACTICAL ADS-B v2");
  u8g2.sendBuffer();
  delay(1500);

  connectWiFi();

  // NTP time (timezone from config). Non-blocking; screens show --:-- until set.
  configTime(TIMEZONE, "pool.ntp.org", "time.google.com");

  nearest.valid = false;
}

void loop() {
  uint32_t now = millis();

  if (!firstFetchDone || now - lastPoll >= UPDATE_INTERVAL_MS) {
    if (WiFi.status() != WL_CONNECTED) connectWiFi();
    fetchAircraft();
    if (nearest.valid && strcmp(routeInfo.callsign, nearest.callsign) != 0)
      fetchRoute(nearest.callsign);
    lastPoll = now;
    firstFetchDone = true;
  }

  if (!firstWeatherDone || now - lastWeatherPoll >= WEATHER_INTERVAL_MS) {
    if (WiFi.status() == WL_CONNECTED) fetchWeather();
    lastWeatherPoll = now;
    firstWeatherDone = true;
  }

  // Hold TARGET twice as long when the contact is a rotorcraft -- that is the
  // screen carrying the alert, and it is worth actually reading.
  uint32_t dwell = SCREEN_SWAP_MS[screen];
  if (screen == SCR_TARGET && nearestIsRotor()) dwell *= 2;

  if (now - lastScreenSwap >= dwell) {
    screen = (screen + 1) % NUM_SCREENS;
    lastScreenSwap = now;
  }

  render();
  buzzerService();   // non-blocking: emits any queued chirp that is now due
  delay(33);   // ~30 fps: smooth radar sweep + fast-ticking clock
}
