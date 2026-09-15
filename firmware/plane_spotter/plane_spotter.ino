/*
 * ESP8266 Plane Spotter
 * --------------------------------------------------------------------------
 * Shows the aircraft currently flying closest to your home on a 0.96" SSD1306
 * I2C OLED, plus a bunch of nerdy statistics. Live ADS-B data is pulled from
 * the free OpenSky Network REST API.
 *
 * Board   : any ESP8266 (NodeMCU v2/v3, Wemos D1 mini, ...)
 * Display : 128x64 OLED, I2C or SPI. Two controllers are supported and both
 *           are the same pixel grid, so the layouts are identical:
 *             - SSD1309, 2.42" (the current build -- DISPLAY_SSD1309 1)
 *             - SSD1306, 0.96" (DISPLAY_SSD1309 0)
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
#include <StreamString.h>
#include <ArduinoJson.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>

#include "config.h"

// Range inside which a rotorcraft or military contact outranks closer civil
// traffic for the TARGET / INTEL / WEAPONS pages. Beyond it, closest wins as
// before. Overridable from config.h; the default is a little past
// BUZZER_RANGE_KM so a contact still holds the pages as it crosses the gate.
#ifndef TARGET_PRIORITY_RANGE_KM
#define TARGET_PRIORITY_RANGE_KM  17.0
#endif

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

// Controller, independent of the bus above. Both panels are 128x64, so every
// layout is unchanged between them -- the 2.42" is the same pixel grid at ~2.5x
// the linear size, not more pixels. All four bus/controller combinations
// compile; check them when touching this block.
//   0 = SSD1306, the 0.96" panel
//   1 = SSD1309, the 2.42" panel (HiLetgo and similar, I2C 4-pin or SPI 7-pin)
#define DISPLAY_SSD1309 1

// SSD1309 init variant. These modules ship with one of two init sequences and
// the datasheet does not tell you which. If the panel stays dark but the I2C
// probe finds it (see the [oled] boot log), flip this before suspecting wiring.
//   0 = NONAME0 (try first), 1 = NONAME2
#define SSD1309_NONAME2 0

// Reset line. The 0.96" panel has none, so U8g2 gets U8X8_PIN_NONE. The 2.42"
// module breaks RES out even in I2C mode and generally wants a real reset
// pulse -- set PIN_OLED_RST_I2C in config.h to the GPIO you wired it to, or
// leave it at -1 to run without one.
#if DISPLAY_SSD1309 && PIN_OLED_RST_I2C >= 0
  #define OLED_RESET_PIN PIN_OLED_RST_I2C
#else
  #define OLED_RESET_PIN U8X8_PIN_NONE
#endif

#if DISPLAY_I2C
// Constructor args: (rotation, reset, clock/SCL, data/SDA).
#  if DISPLAY_SSD1309
#    if SSD1309_NONAME2
U8G2_SSD1309_128X64_NONAME2_F_HW_I2C u8g2(U8G2_R0,
                                          /*reset=*/ OLED_RESET_PIN,
                                          /*clock=*/ PIN_OLED_SCL,
                                          /*data=*/  PIN_OLED_SDA);
#    else
U8G2_SSD1309_128X64_NONAME0_F_HW_I2C u8g2(U8G2_R0,
                                          /*reset=*/ OLED_RESET_PIN,
                                          /*clock=*/ PIN_OLED_SCL,
                                          /*data=*/  PIN_OLED_SDA);
#    endif
#  else
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0,
                                         /*reset=*/ U8X8_PIN_NONE,
                                         /*clock=*/ PIN_OLED_SCL,
                                         /*data=*/  PIN_OLED_SDA);
#  endif
#else
// SPI: SCK=GPIO14 and MOSI=GPIO13 are fixed by hardware SPI; CS/DC/RST are not.
#  if DISPLAY_SSD1309
#    if SSD1309_NONAME2
U8G2_SSD1309_128X64_NONAME2_F_4W_HW_SPI u8g2(U8G2_R0, PIN_OLED_CS, PIN_OLED_DC, PIN_OLED_RST);
#    else
U8G2_SSD1309_128X64_NONAME0_F_4W_HW_SPI u8g2(U8G2_R0, PIN_OLED_CS, PIN_OLED_DC, PIN_OLED_RST);
#    endif
#  else
U8G2_SSD1306_128X64_NONAME_F_4W_HW_SPI u8g2(U8G2_R0,
                                            PIN_OLED_CS,
                                            PIN_OLED_DC,
                                            PIN_OLED_RST);
#  endif
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

// The *target*: the contact TARGET / INTEL / WEAPONS describe and the radar
// circles. Named `nearest` for history, but since the priority change it is
// the closest contact *within the highest priority tier present*, not the
// closest full stop -- see the selection in fetchAircraft().
Aircraft nearest;

// Types for the WEAPONS SYSTEM page. Declared up here because the Arduino
// builder emits function prototypes above the sketch body, so anything used in
// a signature must already be a complete type by then.
enum class AirframeClass : uint8_t { FIXED_WING, HELICOPTER, UAV, UNKNOWN };
enum class AltitudeBand  : uint8_t { VERY_LOW, LOW_ALT, MEDIUM, MED_HIGH, HIGH_ALT, UNKNOWN };
enum class ThreatLevel   : uint8_t { LOW_THREAT, MED_THREAT, HIGH_THREAT, EXTREME_THREAT, UNKNOWN };
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
//
// Positions are flat east/north offsets from home in km, with the velocity
// already resolved onto the same axes (km/s). The radar redraws 30x a second
// and used to re-run the great-circle projection, haversine and bearing for
// every blip every frame -- roughly 26 software-double transcendental calls
// each, which on an 80 MHz core with no FPU was most of the frame budget in a
// busy sky. In this form a frame costs one sqrt and one atan2 per blip, and
// the pixel position is a scale of x/y with no trig at all. The flat-earth
// error over the 30 km ring is well under a pixel.
//
// The trig that remains is the double sin/cos/atan2 already linked for the
// geo helpers, on purpose: the float variants drag ~1 KB of libm
// argument-reduction tables into DRAM (two_over_pi and friends), which on this
// chip is real RAM, for a per-call saving that does not matter at one call
// per blip per frame.
struct Blip {
  float   x, y;     // km east / north of home at lastDataMs
  float   vx, vy;   // km/s east / north (0 when speed or track is unknown)
  uint8_t cat;      // effective emitter category (8 = rotorcraft)
  bool    loiter;   // rotorcraft that has held station (see HeliTrack)
  bool    mil;      // icao24 in a military block, or a military type code
};
const uint8_t MAX_BLIPS = 20;
Blip     blips[MAX_BLIPS];
uint8_t  blipCount  = 0;
int8_t   targetBlip = -1;  // index into blips[] of `nearest`, or -1 if it did not fit
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
  // Alert bookkeeping, separate from "seen". Every voice is gated on
  // BUZZER_RANGE_KM at the moment it would sound, and the search-box edge is
  // always outside that gate (a 0.2 deg box is 17-22 km from home; the gate is
  // 15 km), so for anything that flies in, "first sighting" and "first
  // in-range sighting" are different polls. Gating on the first-seen poll
  // alone meant the acquisition voice could only ever sound for a contact
  // that popped into existence already close. These remember whether the
  // voice has actually sounded, so it fires on the first poll the contact is
  // close enough, whenever that is.
  bool     acquireAlerted; // acquisition voice has sounded for this airframe
  bool     loiterAlerted;  // loiter voice has sounded for this anchor
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

// Route / airline / ETA per callsign (from hexdb.io), cached the same way as
// airframe identity and for the same reason: on an approach path two aircraft
// alternate as nearest faster than the screens cycle, and every swap used to
// cost two fresh TLS round trips (route, then arrival airport) with the
// display frozen throughout. Negatives are cached too -- a GA tail number
// never has a route, and re-asking every poll was pure latency.
struct RouteInfo {
  char     callsign[10];   // which callsign this data is for ("" = free slot)
  char     airline[18];
  char     dep[6];
  char     arr[6];
  bool     haveRoute;
  bool     haveArrPos;
  float    arrLat, arrLon; // float is plenty for an ETA over hundreds of km
  uint32_t touchedMs;      // last hit, for LRU eviction
};
const uint8_t ROUTE_CACHE_N = 8;
RouteInfo routeCache[ROUTE_CACHE_N];

// Identity of the current nearest airframe, resolved by icao24 and cached so a
// lookup only fires when the contact changes. A negative result is cached too
// (icao24 set, haveInfo false), so an unknown airframe is not re-queried every
// poll. See AC_LOOKUP_* in config.h.
struct AircraftInfo {
  char     icao24[8];    // which airframe this describes
  char     reg[10];      // registration / tail number
  char     icaoType[6];  // ICAO type designator, e.g. C172, R22, A321
  bool     haveInfo;     // a tier answered; false is a cached *negative*
  uint8_t  tier;         // which tier answered (0 = none), for the logs
  bool     used;         // slot is occupied
  uint32_t touchedMs;    // last hit, for LRU eviction
};

// Identity cache. This used to be a single record, which meant an airframe was
// re-queried every time it came back around -- and on an approach path aircraft
// cycle through faster than the six screens do, so the same handful of tails
// were being looked up repeatedly. Caching them (negatives included, so an
// unknown icao24 is not retried every poll) makes re-acquisition free and is
// what pays for widening lookups beyond the nearest contact.
AircraftInfo acCache[AC_CACHE_N];

// Airframes waiting to be resolved, filled during the fetch parse and drained
// afterwards -- never during it, because a lookup opens its own TLS client and
// two 16 KB RX buffers must not coexist with the OpenSky one.
struct AcPending { char icao24[8]; float distanceKm; };
AcPending acQueue[AC_QUEUE_N];
uint8_t   acQueueN = 0;

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
   5000,  // WX
   3000,  // SYSTEM -- a glance at the clock and link status is all it needs
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
// Aircraft identity lookup (nearest contact only)
//
// Positions always come from OpenSky. This resolves *who* the nearest contact
// is -- registration and ICAO type designator -- which classifies the airframe
// exactly instead of guessing it from speed and altitude.
//
// Three tiers, cheapest and most reliable first. Each is only consulted if the
// previous missed, and the whole chain only runs when the nearest contact
// changes. Every tier is allowed to fail: the caller falls back to the
// kinematic guess, so no lookup can break the display.
// ---------------------------------------------------------------------------
// ICAO type designators that are rotorcraft. Space-delimited with leading and
// trailing spaces so a padded " CODE " match cannot hit a substring (" B06 "
// must not match "B06T"). Not exhaustive -- covers what actually flies over
// the US -- and anything unmatched falls through to fixed-wing, which is the
// safe default once a type code has resolved at all.
static const char HELI_TYPES[] PROGMEM =
  " R22 R44 R66 B06 B06T B47G B212 B222 B230 B407 B412 B427 B429 B430 B505 "
  "A109 A119 A139 A169 A189 EC20 EC25 EC30 EC35 EC45 EC55 EC75 "
  "AS50 AS55 AS65 AS32 AS35 AS3B S76 S92 S61 S64 S70 H500 H60 UH1 "
  "MD52 MD60 MD90 EH10 NH90 GAZL LYNX PUMA EN48 EXPL ";

// Uncrewed types that show up on ADS-B. Short list; most UAVs do not squawk.
static const char UAV_TYPES[] PROGMEM = " RQ4 MQ9 MQ1 Q4 SHDW UAV ";

// Military ICAO type designators, used to *confirm* a military contact once a
// type code has resolved. It cannot be the primary signal: the identity lookup
// only resolves contacts inside the rotorcraft envelope, so a C-130 at 400 kt
// is never looked up and would never match. Detection is by icao24 block below.
static const char MIL_TYPES[] PROGMEM =
  " C130 C30J C130J LC30 EC30 KC30 C5M C5 C17 C27J "
  "KC35 K35R KC10 KC46 B52 B1 B2 "
  "F15 F16 F18 F22 F35 A10 AV8B T38 T6 T45 "
  "E3TF E3CF E6 E2 C2 P8 P3 U2 RC35 "
  "V22 H60 H64 H47 UH1 H53 H72 ";

// ICAO 24-bit address blocks allocated to military operators.
//
// This is the *primary* signal precisely because it is free: the address is in
// every state vector already, so it costs one integer compare and works on the
// fast, high traffic the identity lookup will never touch.
//
// Only the US block is listed. That is deliberate rather than lazy -- this
// device sits under the Washington DC area, where Andrews traffic is the
// realistic case, and a wrong range is worse than a missing one because it
// paints civil aircraft as military. Other nations' blocks are published and
// easy to add here, but none has been checked against real traffic from this
// spot, so they stay out until there is a reason. Ranges are inclusive.
struct HexRange { uint32_t lo, hi; };
static const HexRange MIL_HEX[] PROGMEM = {
  { 0xADF7C8, 0xAFFFFF },   // United States military
};

// Parse a 6-hex-digit icao24. Returns false on anything malformed, which is
// treated as "not military" -- never guess upward on bad input.
bool parseHex24(const char* hex, uint32_t& out) {
  if (hex == nullptr) return false;
  uint32_t v = 0;
  uint8_t  n = 0;
  for (; hex[n]; n++) {
    char c = hex[n];
    uint8_t d;
    if      (c >= '0' && c <= '9') d = c - '0';
    else if (c >= 'a' && c <= 'f') d = c - 'a' + 10;
    else if (c >= 'A' && c <= 'F') d = c - 'A' + 10;
    else return false;
    v = (v << 4) | d;
  }
  if (n != 6) return false;
  out = v;
  return true;
}

bool isMilitaryHex(const char* hex) {
  uint32_t v;
  if (!parseHex24(hex, v)) return false;
  for (uint8_t i = 0; i < sizeof(MIL_HEX) / sizeof(MIL_HEX[0]); i++) {
    uint32_t lo = pgm_read_dword(&MIL_HEX[i].lo);
    uint32_t hi = pgm_read_dword(&MIL_HEX[i].hi);
    if (v >= lo && v <= hi) return true;
  }
  return false;
}

// Exact token match against a space-delimited PROGMEM list. Walks the list a
// byte at a time rather than copying it to a buffer -- it is ~250 bytes and
// would otherwise want a stack buffer sized to match, which silently breaks
// the tail of the list if the two ever drift apart.
bool typeInList(const char* pgmList, const char* icaoType) {
  if (icaoType == nullptr || icaoType[0] == '\0') return false;
  char    tok[8];
  uint8_t n = 0;
  for (const char* p = pgmList;; p++) {
    char c = (char)pgm_read_byte(p);
    if (c == ' ' || c == '\0') {
      if (n) { tok[n] = '\0'; if (strcasecmp(tok, icaoType) == 0) return true; }
      n = 0;
      if (c == '\0') break;
    } else if (n < sizeof(tok) - 1) {
      tok[n++] = c;
    }
  }
  return false;
}

// Military if the address block says so, or if a resolved type code does. The
// address is the detector; the type code only ever adds to it, so a contact
// outside the listed blocks can still be caught once identity resolves.
bool isMilitary(const char* hex, const char* icaoType) {
  return isMilitaryHex(hex) || typeInList(MIL_TYPES, icaoType);
}

#if AC_LOOKUP_ENABLE
// Copy a small field out of a JSON doc into a fixed buffer, trimmed.
void copyField(char* dst, size_t n, const char* src) {
  if (src == nullptr) { dst[0] = '\0'; return; }
  while (*src == ' ') src++;
  strncpy(dst, src, n - 1);
  dst[n - 1] = '\0';
  for (int i = strlen(dst) - 1; i >= 0 && dst[i] == ' '; i--) dst[i] = '\0';
}

// Tier 1: hexdb.io. Already used for routes, so no new dependency.
bool acLookupHexdb(const char* hex, AircraftInfo& out) {
  String payload;
  if (!httpGetString(String("https://hexdb.io/api/v1/aircraft/") + hex, payload)) return false;
  JsonDocument d;
  if (deserializeJson(d, payload)) return false;
  const char* reg = d["Registration"] | "";
  const char* typ = d["ICAOTypeCode"] | "";
  if (!reg[0] && !typ[0]) return false;
  copyField(out.reg, sizeof(out.reg), reg);
  copyField(out.icaoType, sizeof(out.icaoType), typ);
  return true;
}

// Tier 2: adsbdb.com. Independent database, better on US general aviation.
bool acLookupAdsbdb(const char* hex, AircraftInfo& out) {
  String payload;
  if (!httpGetString(String("https://api.adsbdb.com/v0/aircraft/") + hex, payload)) return false;
  JsonDocument d;
  if (deserializeJson(d, payload)) return false;
  JsonObject a = d["response"]["aircraft"];
  if (a.isNull()) return false;
  const char* reg = a["registration"] | "";
  const char* typ = a["icao_type"] | "";
  if (!reg[0] && !typ[0]) return false;
  copyField(out.reg, sizeof(out.reg), reg);
  copyField(out.icaoType, sizeof(out.icaoType), typ);
  return true;
}

#if AC_LOOKUP_TIER3
// Tier 3: adsb.lol. Community-run, and a live-feed query rather than a
// database -- it only knows aircraft currently airborne, which is exactly when
// one is our nearest contact. Plain HTTP, so no TLS buffer. Only reached when
// both databases miss.
bool acLookupAdsbLol(const char* hex, AircraftInfo& out) {
  String payload;
  if (!httpGetStringPlain(String("http://api.adsb.lol/v2/hex/") + hex, payload)) return false;
  JsonDocument d;
  if (deserializeJson(d, payload)) return false;
  JsonArray ac = d["ac"].as<JsonArray>();
  if (ac.isNull() || ac.size() == 0) return false;
  JsonObject a = ac[0];
  const char* reg = a["r"] | "";
  const char* typ = a["t"] | "";
  if (!reg[0] && !typ[0]) return false;
  copyField(out.reg, sizeof(out.reg), reg);
  copyField(out.icaoType, sizeof(out.icaoType), typ);
  return true;
}
#endif

// Cache entry for `hex`, or nullptr if this airframe has never been resolved.
// A hit here counts as known even when haveInfo is false -- that is a cached
// negative and must suppress a retry just as firmly as a positive.
AircraftInfo* acCacheFind(const char* hex) {
  if (hex == nullptr || hex[0] == '\0') return nullptr;
  for (uint8_t i = 0; i < AC_CACHE_N; i++) {
    if (acCache[i].used && strcmp(acCache[i].icao24, hex) == 0) {
      acCache[i].touchedMs = millis();
      return &acCache[i];
    }
  }
  return nullptr;
}

// A slot to write `hex` into: its existing entry, else a free slot, else the
// least recently touched one.
AircraftInfo* acCacheSlot(const char* hex) {
  AircraftInfo* hit = acCacheFind(hex);
  if (hit) return hit;

  AircraftInfo* victim = &acCache[0];
  for (uint8_t i = 0; i < AC_CACHE_N; i++) {
    if (!acCache[i].used) { victim = &acCache[i]; break; }
    if ((int32_t)(acCache[i].touchedMs - victim->touchedMs) < 0) victim = &acCache[i];
  }
  return victim;
}

bool acKnown(const char* hex) { return acCacheFind(hex) != nullptr; }

void fetchAircraftInfo(const char* hex) {
  if (hex == nullptr || hex[0] == '\0') return;
  if (acCacheFind(hex)) return;                 // already resolved, or a negative

  AircraftInfo* e = acCacheSlot(hex);
  copyField(e->icao24, sizeof(e->icao24), hex);
  e->reg[0] = e->icaoType[0] = '\0';
  e->haveInfo = false;
  e->tier     = 0;
  e->used     = true;
  e->touchedMs = millis();

  uint32_t t0 = millis();
  if      (acLookupHexdb(hex, *e))  { e->haveInfo = true; e->tier = 1; }
  else if (acLookupAdsbdb(hex, *e)) { e->haveInfo = true; e->tier = 2; }
#if AC_LOOKUP_TIER3
  else if (acLookupAdsbLol(hex, *e)){ e->haveInfo = true; e->tier = 3; }
#endif

  // Duration matters as much as the result: these are synchronous HTTPS calls
  // on the same thread as the 30 fps render loop, so it is what bounds how many
  // can run per poll (AC_LOOKUP_MAX_PER_POLL).
  Serial.printf("[acid] %s -> %s %s (tier %u) %lums heap=%u\n", hex,
                e->haveInfo ? e->reg      : "?",
                e->haveInfo ? e->icaoType : "?",
                e->tier, (unsigned long)(millis() - t0), ESP.getFreeHeap());
}

// Queue an airframe for resolution after the fetch finishes. Closest wins when
// the queue is full, since a distant contact matters least and may well leave
// the box before it is ever the nearest.
void acEnqueue(const char* hex, float distanceKm) {
  if (hex == nullptr || hex[0] == '\0' || acKnown(hex)) return;
  for (uint8_t i = 0; i < acQueueN; i++)
    if (strcmp(acQueue[i].icao24, hex) == 0) return;

  if (acQueueN < AC_QUEUE_N) {
    copyField(acQueue[acQueueN].icao24, sizeof(acQueue[0].icao24), hex);
    acQueue[acQueueN].distanceKm = distanceKm;
    acQueueN++;
    return;
  }
  uint8_t worst = 0;
  for (uint8_t i = 1; i < AC_QUEUE_N; i++)
    if (acQueue[i].distanceKm > acQueue[worst].distanceKm) worst = i;
  if (distanceKm < acQueue[worst].distanceKm) {
    copyField(acQueue[worst].icao24, sizeof(acQueue[0].icao24), hex);
    acQueue[worst].distanceKm = distanceKm;
  }
}

// Resolve up to AC_LOOKUP_MAX_PER_POLL queued airframes, nearest first. Called
// after fetchAircraft() has returned and its TLS client is gone.
void acDrainQueue() {
  uint8_t done = 0;
  while (acQueueN > 0 && done < AC_LOOKUP_MAX_PER_POLL) {
    uint8_t best = 0;
    for (uint8_t i = 1; i < acQueueN; i++)
      if (acQueue[i].distanceKm < acQueue[best].distanceKm) best = i;

    char hex[8];
    copyField(hex, sizeof(hex), acQueue[best].icao24);
    acQueue[best] = acQueue[--acQueueN];        // compact
    fetchAircraftInfo(hex);
    done++;
  }
  acQueueN = 0;   // anything left is stale next poll anyway; it will re-queue
}

// The resolved type code for `hex`, or "" when we have nothing for it.
const char* acTypeFor(const char* hex) {
  const AircraftInfo* e = acCacheFind(hex);
  return (e && e->haveInfo) ? e->icaoType : "";
}
const char* acRegFor(const char* hex) {
  const AircraftInfo* e = acCacheFind(hex);
  return (e && e->haveInfo) ? e->reg : "";
}

#else
inline void fetchAircraftInfo(const char*) {}
inline void acEnqueue(const char*, float)  {}
inline void acDrainQueue()                 {}
inline bool acKnown(const char*)           { return false; }
inline const char* acTypeFor(const char*)  { return ""; }
inline const char* acRegFor(const char*)   { return ""; }
#endif

// ---------------------------------------------------------------------------
// Aircraft type + rotorcraft tracking
// ---------------------------------------------------------------------------

// OpenSky usually leaves the emitter category at 0 (unknown), so fall back to a
// rough guess from altitude + ground speed. Real category data always wins.
// Estimated types are flagged with '~' on screen.
int categoryFrom(int cat, bool onGround, float velocityMs, float altitudeM) {
  // Categories 0 and 1 are both "no information" in the ADS-B emitter enum --
  // 0 is the field absent, 1 is the transponder explicitly saying it has none.
  // Only 2 and up name an actual airframe class, so 1 must fall through to the
  // guess rather than being echoed back as though it were identity.
  if (cat > 1)  return cat;                   // real data
  if (onGround) return 0;
  // A missing field is not a slow, low one. OpenSky leaves velocity and both
  // altitudes null often enough that defaulting them to zero made every such
  // contact look like a hovering helicopter -- a screen full of crosses, and
  // airliners latching into helis[]. Without real kinematics, decline to guess.
  if (!isfinite(velocityMs) || !isfinite(altitudeM)) return 0;
  float kmh = velocityMs * 3.6f;
  if (kmh < 120 && altitudeM < 2200) return 8;  // slow & low -> guess helicopter
  if (kmh < 300 && altitudeM < 5000) return 3;  // medium      -> guess small plane
  return 4;                                     // fast / high -> airliner
}

bool isRotor(int cat) { return cat == 8; }

// Apply a resolved ICAO type code to a category. A type code is hard identity
// and outranks both the emitter category and the kinematic guess: it puts a
// cross on a helicopter transiting above the 120 km/h guess threshold and
// takes a wrong one off a slow fixed-wing on approach. One rule for the blips
// at fetch time and for the target on the pages -- the target used to get the
// bare guess, so a Black Hawk at 205 km/h drew as "~Small" beside a banner
// that correctly said MIL ROTOR.
int identityCategory(int cat, const char* icaoType) {
  if (icaoType == nullptr || icaoType[0] == '\0') return cat;
  if (typeInList(HELI_TYPES, icaoType)) return 8;
  if (typeInList(UAV_TYPES,  icaoType)) return 14;
  return cat == 8 ? 3 : cat;      // guessed heli, identity says otherwise
}

// Fold one rotorcraft sighting into the tracking table. Returns true if this
// airframe currently counts as loitering. Called once per rotorcraft per fetch.
// `distanceKm` is the contact's range from home, already computed by the
// caller. It gates both voices on the first poll the contact is inside
// BUZZER_RANGE_KM, not only on the poll it appeared -- see HeliTrack.
bool trackRotorcraft(const char* icao, double lat, double lon, double distanceKm) {
  uint32_t now = millis();
  if (icao == nullptr || icao[0] == '\0') return false;
  bool inRange = distanceKm <= BUZZER_RANGE_KM;

  for (uint8_t i = 0; i < heliCount; i++) {
    if (strcmp(helis[i].icao24, icao) != 0) continue;
    helis[i].lastSeenMs = now;
    if (haversineKm(helis[i].refLat, helis[i].refLon, lat, lon) > LOITER_RADIUS_KM) {
      helis[i].refLat        = lat;    // moved on: re-anchor, it is transiting
      helis[i].refLon        = lon;
      helis[i].sinceMs       = now;
      helis[i].loitering     = false;
      helis[i].loiterAlerted = false;  // a fresh anchor is a fresh orbit
    } else if (!helis[i].loitering &&
               (uint32_t)(now - helis[i].sinceMs) >= LOITER_MIN_MS) {
      helis[i].loitering = true;
      Serial.printf("[heli] %s loitering: %lu min within %.1f km\n",
                    icao, (unsigned long)((now - helis[i].sinceMs) / 60000UL),
                    (double)LOITER_RADIUS_KM);
    }

    // One voice per poll. If both fall due together -- a contact that latches
    // and closes inside the gate on the same fetch -- loiter is the more
    // specific fact, and the acquisition is marked done rather than queued
    // behind it, since buzzerChirp() would drop the second anyway.
    if (inRange) {
      bool acquireDue = !helis[i].acquireAlerted;
      bool loiterDue  = helis[i].loitering && !helis[i].loiterAlerted;
      helis[i].acquireAlerted = true;
      if (loiterDue) { helis[i].loiterAlerted = true; buzzerLoiter(); }
      else if (acquireDue) buzzerAcquire();
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
  helis[slot].refLat         = lat;
  helis[slot].refLon         = lon;
  helis[slot].sinceMs        = now;
  helis[slot].lastSeenMs     = now;
  helis[slot].loitering      = false;
  helis[slot].acquireAlerted = inRange;
  helis[slot].loiterAlerted  = false;
  Serial.printf("[heli] new contact %s\n", icao);
  if (inRange) buzzerAcquire();
  return false;
}

// Military contacts already seen, so the log line fires on arrival rather than
// every poll for as long as one is overhead. Same shape as helis[] and for the
// same reason: blips[] is rebuilt each fetch and cannot remember anything.
// Sizing is MAX_MIL in config.h -- set for a formation, not the typical case,
// because overflow re-announces every poll.
struct MilTrack {
  char     icao24[8];
  uint32_t lastSeenMs;
  bool     alerted;     // the voice has sounded -- separate from "seen" for the
                        // reason given at HeliTrack
};
MilTrack milSeen[MAX_MIL];
uint8_t  milCount = 0;

// Fold one military sighting in. Returns true only on the poll the airframe
// first appears, for the log. `alert` is set on the poll the voice should
// sound: the first one with the contact inside BUZZER_RANGE_KM. Those are
// usually different polls -- a contact flying in is first seen at the box
// edge, 17-22 km out, and only later closes inside the 15 km gate. Gating the
// voice on the first-seen poll alone meant it could never sound for anything
// that did not pop into existence already close, which in practice was almost
// everything.
bool trackMilitary(const char* icao, double distanceKm, bool& alert) {
  alert = false;
  if (icao == nullptr || icao[0] == '\0') return false;
  uint32_t now = millis();
  bool inRange = distanceKm <= BUZZER_RANGE_KM;

  for (uint8_t i = 0; i < milCount; i++) {
    if (strcmp(milSeen[i].icao24, icao) == 0) {
      milSeen[i].lastSeenMs = now;
      if (inRange && !milSeen[i].alerted) { milSeen[i].alerted = true; alert = true; }
      return false;                       // already announced
    }
  }

  uint8_t slot;
  if (milCount < MAX_MIL) {
    slot = milCount++;
  } else {                                // evict the stalest
    slot = 0;
    for (uint8_t i = 1; i < MAX_MIL; i++)
      if ((int32_t)(milSeen[i].lastSeenMs - milSeen[slot].lastSeenMs) < 0) slot = i;
  }
  strncpy(milSeen[slot].icao24, icao, sizeof(milSeen[0].icao24) - 1);
  milSeen[slot].icao24[sizeof(milSeen[0].icao24) - 1] = '\0';
  milSeen[slot].lastSeenMs = now;
  milSeen[slot].alerted    = inRange;
  alert = inRange;
  return true;
}

void expireMilitary() {
  uint32_t now = millis();
  uint8_t  w   = 0;
  for (uint8_t i = 0; i < milCount; i++) {
    if ((uint32_t)(now - milSeen[i].lastSeenMs) < MIL_EXPIRE_MS) {
      if (w != i) milSeen[w] = milSeen[i];
      w++;
    } else {
      Serial.printf("[mil] %s lost\n", milSeen[i].icao24);
    }
  }
  milCount = w;
}

// Drop an airframe from the loiter table because identity has since proved it
// is not a rotorcraft.
//
// This happens for real: an aircraft on approach is slow and low, which is all
// categoryFrom() has to go on, so a Cessna at 115 km/h and 190 m gets guessed
// as a helicopter and anchored here a poll before the type code arrives to say
// C172. Without this the false entry sits in a MAX_HELI slot until
// HELI_EXPIRE_MS -- five minutes of a four-slot table held by a light single,
// which could crowd out an actual helicopter.
void dropRotorcraft(const char* icao, const char* icaoType) {
  if (icao == nullptr || icao[0] == '\0') return;
  for (uint8_t i = 0; i < heliCount; i++) {
    if (strcmp(helis[i].icao24, icao) != 0) continue;
    Serial.printf("[heli] %s reclassified as %s, dropped\n",
                  icao, icaoType[0] ? icaoType : "?");
    helis[i] = helis[--heliCount];      // order does not matter here
    return;
  }
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

  // Must be set before the request goes out -- it changes the request line.
  // HTTP/1.0 means the server cannot use Transfer-Encoding: chunked, which is
  // what was truncating this fetch; see the note at the parse below.
  https.useHTTP10(true);

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

  // Parse straight off the socket. Nothing buffers the whole body.
  //
  // The old getString() path was truncating ~15% of polls, silently, mid-token.
  // Root cause is memory pressure inside the chunked decoder: the HTTP/1.1
  // response was Transfer-Encoding: chunked, and the core's chunked loop
  // allocates a fresh String for every chunk header via readStringUntil('\n')
  // -- all while the 16 KB TLS RX buffer leaves just ~8 KB free and ~5.4 KB
  // contiguous. When an allocation there fails, bytes-written stops matching
  // bytes-declared and the loop bails with HTTPC_ERROR_STREAM_WRITE (-10),
  // handing back a partial body. getString() returns `const String&` and has
  // no error channel, so the only symptom downstream was a JSON IncompleteInput.
  //
  // useHTTP10(true) (set before GET, above) removes the whole failure mode
  // rather than working around it: HTTP/1.0 has no chunked encoding, so there
  // are no chunk headers and no per-chunk allocations. That is also what makes
  // streaming safe here -- getStream() on the *chunked* response would have fed
  // raw hex length markers to the parser. With chunking gone, ArduinoJson reads
  // the socket directly and the ~2 KB body String disappears from the heap.
  //
  // Note OpenSky sends no Content-Length even on HTTP/1.0 -- getSize() stays
  // -1 and the server closes the connection to signal end-of-body. Streaming
  // does not need the length, so that is fine; do not "restore" getString()
  // here, and see CLAUDE.md before touching any of this.
  JsonDocument doc;
  DeserializationError err = deserializeJson(
      doc, https.getStream(), DeserializationOption::Filter(filter));
  int bodyLen = https.getSize();
  https.end();

  // bodyLen is always -1 (no Content-Length, see above), so don't log it as if
  // it measured anything -- the parse either succeeded or errors just below.
  Serial.printf("[fetch] streamed ok, heap=%u\n", ESP.getFreeHeap());

  if (err) {
    // IncompleteInput here now means the socket really did end early, not that
    // a buffer failed to grow -- there is no intermediate buffer any more.
    Serial.printf("[fetch] JSON error: %s (declared %d B)\n", err.c_str(), bodyLen);
    stats.requestsFail++;
    return false;
  }

  JsonArray states = doc["states"].as<JsonArray>();
  Aircraft best;
  best.valid      = false;
  best.distanceKm = 1e9;
  uint8_t  bestPrio  = 0;
  double   minD      = 1e9;   // true closest, for the SYSTEM statistic
  uint16_t count     = 0;
  uint8_t  rotorSeen = 0;
  uint8_t  milSeen4Poll = 0;
  blipCount = 0;

  for (JsonArray s : states) {
    if (s.isNull() || s[5].isNull() || s[6].isNull()) continue;
    double lon = s[5].as<double>();
    double lat = s[6].as<double>();
    double d   = haversineKm(HOME_LAT, HOME_LON, lat, lon);
    double brg = bearingDeg(HOME_LAT, HOME_LON, lat, lon);
    count++;
    if (d < minD) minD = d;

    // Resolve the type here, while the full state vector is in hand: the radar
    // redraws far too often to re-derive it per frame.
    // NAN, not 0, for anything OpenSky left null -- that is the sentinel the
    // threat/weapons layer already tests with isfinite(), and it keeps
    // "unknown" distinguishable from "stationary at sea level".
    bool  onGround = s[8] | false;
    float velMs    = s[9].isNull() ? NAN : s[9].as<float>();
    float altM     = s[13].isNull() ? (s[7].isNull() ? NAN : s[7].as<float>())
                                    : s[13].as<float>();
    int   cat      = categoryFrom(s[17] | 0, onGround, velMs, altM);

    // A resolved ICAO type is hard identity and outranks the kinematic guess --
    // for every contact now, not just the nearest. This is what puts a cross on
    // a helicopter transiting above the 120 km/h guess threshold, and equally
    // what takes a wrong cross *off* a slow fixed-wing on approach.
    const char* hexId = s[0] | "";
    const char* known = acTypeFor(hexId);
    if (known[0]) {
      int fixed = identityCategory(cat, known);
      if (cat == 8 && fixed != 8) dropRotorcraft(hexId, known);   // retract the anchor too
      cat = fixed;
    } else if (!onGround &&
               isfinite(velMs) && velMs * 3.6f < AC_LOOKUP_ENVELOPE_KMH &&
               isfinite(altM)  && altM         < AC_LOOKUP_ENVELOPE_M &&
               d <= AC_LOOKUP_RANGE_KM) {
      // Plausibly a rotorcraft and close enough to matter: worth an identity
      // lookup once this fetch is done and its TLS client has gone away.
      acEnqueue(hexId, (float)d);
    }

    // Military: free to evaluate, so every contact gets checked regardless of
    // speed, altitude or range -- unlike identity, which is gated.
    bool mil = isMilitary(hexId, known);

    // A missing track is NAN like the other kinematics: 0 would dead-reckon
    // the blip due north and point the TARGET arrow there.
    float trackDeg = s[10].isNull() ? NAN : s[10].as<float>();

#if LOG_BLIP_DUMP
    // One line per contact so a rotorcraft or military contact seen on
    // FlightRadar can be matched against what the gates actually decided --
    // the difference between "rotor=0, no idea why" and a diagnosis. Off by
    // default for the privacy reason documented in config.h: distance-from-home
    // against public aircraft positions trilaterates the device location.
    Serial.printf("[blip] %s %-8s d=%5.1fkm v=%6.1fkmh alt=%7.1fm cat=%2d%s%s\n",
                  (const char*)(s[0] | "------"), (const char*)(s[1] | ""),
                  d,
                  isfinite(velMs) ? velMs * 3.6f : -1.0f,
                  isfinite(altM)  ? altM         : -1.0f,
                  cat, isRotor(cat) ? " <-- ROTOR" : "", mil ? " <-- MIL" : "");
#endif

    if (mil) {
      milSeen4Poll++;
      bool alert;
      if (trackMilitary(hexId, d, alert))
        Serial.printf("[mil] new contact %s %s d=%.1fkm type=%s\n",
                      hexId, (const char*)(s[1] | ""), d, known[0] ? known : "?");
      // Range is gated inside trackMilitary(); quiet hours inside buzzerChirp().
      if (alert) buzzerMilitary();
    }

    bool loiter = false;
    if (isRotor(cat)) {
      rotorSeen++;
      loiter = trackRotorcraft(hexId, lat, lon, d);
    }

    int8_t thisBlip = -1;
    if (blipCount < MAX_BLIPS) {
      thisBlip = (int8_t)blipCount;
      // Flat east/north km from the range and bearing already in hand, with
      // the velocity resolved onto the same axes so the radar dead-reckons with
      // one multiply per axis. Unknown speed or track means the blip holds
      // still, which is honest -- guessing a heading would creep it wrongly.
      Blip& b = blips[blipCount++];
      double brgRad = deg2rad(brg);
      b.x = (float)(d * sin(brgRad));
      b.y = (float)(d * cos(brgRad));
      if (!onGround && isfinite(velMs) && isfinite(trackDeg) && velMs > 0.0f) {
        double trkRad = deg2rad(trackDeg);
        b.vx = (float)(velMs * sin(trkRad) / 1000.0);
        b.vy = (float)(velMs * cos(trkRad) / 1000.0);
      } else {
        b.vx = b.vy = 0.0f;
      }
      b.cat    = (uint8_t)cat;
      b.loiter = loiter;
      b.mil    = mil;
    }

    // Target selection: priority tier first, then range within the tier. A
    // rotorcraft anywhere in the box outranks a closer airliner -- the special
    // contacts are the point of the device, and plain "closest" was handing
    // TARGET / INTEL / WEAPONS to a 767 on approach while a Black Hawk sat
    // 2 km further out with its banner never shown. Loitering rotorcraft >
    // rotorcraft > military > everything else -- inside
    // TARGET_PRIORITY_RANGE_KM. Without the cap a departing Black Hawk held
    // the pages from 25 km out over airliners passing overhead. Note the tier
    // uses `cat` as resolved *this* poll, so a fast rotorcraft only outranks
    // once its type code has come back (one poll after it is first queued).
    uint8_t prio = isRotor(cat) ? (loiter ? 3 : 2) : (mil ? 1 : 0);
    if (d > TARGET_PRIORITY_RANGE_KM) prio = 0;
    if (prio > bestPrio || (prio == bestPrio && d < best.distanceKm)) {
      bestPrio        = prio;
      targetBlip      = thisBlip;
      best.distanceKm = d;
      best.lat        = lat;
      best.lon        = lon;
      best.bearingDeg = brg;
      best.onGround   = onGround;
      best.category   = s[17] | 0;
      best.altitudeM  = altM;       // NAN when OpenSky left both altitudes null
      best.velocityMs = velMs;      // ditto; isfinite() guards downstream rely on it
      best.trackDeg   = trackDeg;
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
  expireMilitary();


  stats.inView       = count;
  if (count > stats.maxInView) stats.maxInView = count;
  stats.requestsOk++;
  stats.lastUpdateMs = millis();
  lastDataMs         = millis();

  nearest = best;
  if (!nearest.valid) targetBlip = -1;
  if (count > 0 && minD < stats.closestEver) stats.closestEver = minD;

  Serial.printf("[fetch] inView=%u blips=%u rotor=%u target=%s prio=%u cat=%d dist=%.1fkm valid=%d heap=%u\n",
                count, blipCount, rotorSeen, nearest.valid ? nearest.callsign : "-",
                bestPrio, nearest.valid ? nearest.category : -1,
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

  // Same shape as the OpenSky fetch, for the same reason: HTTP/1.0 rules out
  // chunked encoding, which is what let getString() hand back a silently
  // truncated body under TLS-buffer memory pressure -- and with chunking gone
  // the parser can read the socket directly, with no body String at all.
  https.useHTTP10(true);
  int code = https.GET();
  Serial.printf("[wx] HTTP %d\n", code);
  if (code != HTTP_CODE_OK) { https.end(); return false; }

  JsonDocument filter;
  filter["current"] = true;
  filter["hourly"]["temperature_2m"] = true;
  filter["hourly"]["weather_code"]   = true;
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, https.getStream(),
                                             DeserializationOption::Filter(filter));
  https.end();
  if (err) { Serial.printf("[wx] JSON error: %s\n", err.c_str()); return false; }
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
// Plain HTTP, for endpoints that do not require TLS. Worth a separate helper:
// it skips the 16 KB TLS RX buffer entirely, which is most of the per-request
// heap cost on this chip.
bool httpGetStringPlain(const String& url, String& out) {
  WiFiClient client;
  HTTPClient http;
  http.setReuse(false);
  if (!http.begin(client, url)) return false;
  int code = http.GET();
  if (code != HTTP_CODE_OK) { http.end(); return false; }
  out = http.getString();
  http.end();
  return true;
}

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

// Cached route for `callsign`, or nullptr if it has never been looked up. A
// hit counts as known even with haveRoute false -- that is a cached negative.
RouteInfo* routeFor(const char* callsign) {
  if (callsign == nullptr || callsign[0] == '\0') return nullptr;
  for (uint8_t i = 0; i < ROUTE_CACHE_N; i++) {
    if (routeCache[i].callsign[0] && strcmp(routeCache[i].callsign, callsign) == 0) {
      routeCache[i].touchedMs = millis();
      return &routeCache[i];
    }
  }
  return nullptr;
}

// Does this callsign look like an airline flight -- a three-letter ICAO
// operator designator followed by a flight number? Anything else (an
// N-number, a bare registration, "(no id)") is GA, hexdb has no route for it,
// and the two TLS round trips are skipped: only the offline airline table runs.
bool callsignLooksAirline(const char* cs) {
  if (cs == nullptr || strlen(cs) < 4) return false;
  for (uint8_t i = 0; i < 3; i++)
    if (!isalpha((unsigned char)cs[i])) return false;
  return isdigit((unsigned char)cs[3]) != 0;
}

// Look up departure/arrival airports (and arrival coords for ETA) for a
// callsign and cache the answer. Always fills the airline; route/ETA are
// best-effort and only queried for callsigns that could plausibly have one.
void fetchRoute(const char* callsign) {
  // Slot: a free one, else the least recently touched.
  RouteInfo* r = &routeCache[0];
  for (uint8_t i = 0; i < ROUTE_CACHE_N; i++) {
    if (!routeCache[i].callsign[0]) { r = &routeCache[i]; break; }
    if ((int32_t)(routeCache[i].touchedMs - r->touchedMs) < 0) r = &routeCache[i];
  }

  strncpy(r->airline, airlineName(callsign), sizeof(r->airline) - 1);
  r->airline[sizeof(r->airline) - 1] = '\0';
  r->haveRoute  = false;
  r->haveArrPos = false;
  r->dep[0] = r->arr[0] = '\0';
  strncpy(r->callsign, callsign, sizeof(r->callsign) - 1);
  r->callsign[sizeof(r->callsign) - 1] = '\0';
  r->touchedMs = millis();

  bool query = callsignLooksAirline(callsign);
  if (query) {
    String payload;
    if (httpGetString(String("https://hexdb.io/api/v1/route/icao/") + callsign, payload)) {
      JsonDocument d;
      if (!deserializeJson(d, payload)) {
        const char* rt   = d["route"] | "";
        const char* dash = strchr(rt, '-');
        if (rt[0] && dash) {
          size_t dl = dash - rt;
          if (dl < sizeof(r->dep)) {
            strncpy(r->dep, rt, dl);
            r->dep[dl] = '\0';
            strncpy(r->arr, dash + 1, sizeof(r->arr) - 1);
            r->arr[sizeof(r->arr) - 1] = '\0';
            r->haveRoute = true;
          }
        }
      }
    }

    if (r->haveRoute && r->arr[0]) {
      String ap;
      if (httpGetString(String("https://hexdb.io/api/v1/airport/icao/") + r->arr, ap)) {
        JsonDocument d;
        if (!deserializeJson(d, ap) && !d["latitude"].isNull()) {
          r->arrLat = d["latitude"]  | 0.0f;
          r->arrLon = d["longitude"] | 0.0f;
          r->haveArrPos = true;
        }
      }
    }
  }

  Serial.printf("[route] %s %s %s>%s eta=%s%s\n", callsign, r->airline,
                r->haveRoute ? r->dep : "?",
                r->haveRoute ? r->arr : "?",
                r->haveArrPos ? "yes" : "no",
                query ? "" : " (GA, not queried)");
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
static uint8_t  chirpPrio   = 0;       // priority of the pattern in flight

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

// Queue a pattern. A pattern already in flight keeps the pin unless the new
// one outranks it: the sweep tick is priority 0 and the three alerts are 1, so
// a tick cannot stomp on the tail of a loiter alert, but a military trill
// arriving while a 25 ms tick happens to be sounding replaces the tick instead
// of being lost -- and it was being lost, because fetches land at arbitrary
// points in the sweep. Equal priority is first-come, so alerts never cut each
// other short.
void buzzerChirp(uint8_t prio, uint8_t count, uint16_t freq, uint16_t onMs, uint16_t gapMs) {
  bool busy = chirpsLeft > 0 || chirpOn;
  if (busy && prio <= chirpPrio) return;
  if (buzzerQuietNow()) return;
  if (chirpOn) {                          // cut the outranked tone right now
    noTone(PIN_BUZZER);
    digitalWrite(PIN_BUZZER, BUZZER_IDLE_LEVEL);
    chirpOn = false;
  }
  chirpPrio   = prio;
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

// Stop a tone that is sounding right now but keep the rest of the pattern
// queued. loop() calls this before the fetch block: nothing services the
// buzzer for the seconds a fetch and its identity lookups take, so a chirp that
// happened to be on at that instant used to stay on for all of it -- a
// multi-second howl instead of a 25 ms tick. The pattern resumes on the first
// service call after the fetch returns.
void buzzerPause() {
  if (!chirpOn) return;
  noTone(PIN_BUZZER);
  digitalWrite(PIN_BUZZER, BUZZER_IDLE_LEVEL);
  chirpOn     = false;
  chirpNextMs = millis() + chirpGapMs;
}

// Four voices, deliberately separated on both axes the element can express --
// pitch and rhythm -- because on a single piezo that is all there is to work
// with. Read down the list: pitch falls as the pulses get longer and fewer.
// Military is the odd one out at the top: fastest and highest, a trill rather
// than a beat, so it does not read as "more of the rotorcraft alert".
// All four stay inside the 2-5 kHz band these elements actually project.
// First argument is priority: the tick yields to any alert (see buzzerChirp).
void buzzerSweepBlip()  { buzzerChirp(0, 1, 4000,  25,  40); }  // crisp tick
void buzzerMilitary()   { buzzerChirp(1, 4, 4500,  40,  45); }  // fast high trill
void buzzerAcquire()    { buzzerChirp(1, 2, 3000,  60,  70); }  // two-tone
void buzzerLoiter()     { buzzerChirp(1, 3, 2200, 120, 100); }  // lowest, insistent

#else
inline bool buzzerQuietNow() { return true; }
inline void buzzerChirp(uint8_t, uint8_t, uint16_t, uint16_t, uint16_t) {}
inline void buzzerService()  {}
inline void buzzerPause()    {}
inline void buzzerSweepBlip(){}
inline void buzzerMilitary() {}
inline void buzzerAcquire()  {}
inline void buzzerLoiter()   {}
#endif

// Radar sweep angle for this instant: ~4.3 s per clockwise turn.
float sweepAngleNow() { return fmodf(millis() / 12.0f, 360.0f); }

// Sweep angle on the previous radar frame, so the sweep-crossing blip can tell
// which bearings the beam passed over since last time. loop() re-seeds it as
// the RADAR page comes back around: left stale for a minute it landed inside
// the 30 deg window about one return in twelve and fired a tick for whatever
// sat in that arc.
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
static const char WS_C_NAM[] PROGMEM = "STRYKER";
static const char WS_C_BRN[] PROGMEM = "US ARMY";
static const char WS_C_ROL[] PROGMEM = "POINT DEFENSE";
static const char WS_D_DES[] PROGMEM = "NASAMS";
static const char WS_D_NAM[] PROGMEM = "NASAMS";
static const char WS_D_BRN[] PROGMEM = "US/NORWAY";
static const char WS_D_ROL[] PROGMEM = "AREA DEFENSE";
static const char WS_E_DES[] PROGMEM = "SM-2";
static const char WS_E_NAM[] PROGMEM = "AEGIS";
static const char WS_E_BRN[] PROGMEM = "US NAVY";
static const char WS_E_ROL[] PROGMEM = "FLEET DEFENSE";
static const char WS_F_DES[] PROGMEM = "MADIS";
static const char WS_F_NAM[] PROGMEM = "MADIS";
static const char WS_F_BRN[] PROGMEM = "USMC";
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
// Prefer a resolved ICAO type designator over the emitter category: a type
// code is hard identity (an R22 is a helicopter, always), whereas the category
// is usually absent and then guessed from speed and altitude. Falls back to
// the category path whenever no type code resolved.
AirframeClass classifyAirframeFrom(int category, const char* icaoType) {
  if (icaoType != nullptr && icaoType[0] != '\0') {
    if (typeInList(HELI_TYPES, icaoType)) return AirframeClass::HELICOPTER;
    if (typeInList(UAV_TYPES,  icaoType)) return AirframeClass::UAV;
    return AirframeClass::FIXED_WING;   // a resolved type is a real aircraft
  }
  return classifyAirframe(category);
}

AirframeClass classifyAirframe(int category) {
  switch (category) {
    case 8:  return AirframeClass::HELICOPTER;
    case 14: return AirframeClass::UAV;
    case 2: case 3: case 4: case 5: case 6: case 7: case 9:
             return AirframeClass::FIXED_WING;
    default: return AirframeClass::UNKNOWN;
  }
}

// "Threat" is identity first, geometry second. A rotorcraft is EXTREME
// wherever it is and whatever it is doing -- around here that is the contact
// the whole device exists for, and scoring it LOW because it happened to be
// tracking away was absurd. A military fixed-wing floors at HIGH. Everything
// else is aspect (how directly the contact is tracking over the device)
// tightened by slant range. Everything is friendly regardless -- this drives
// nothing but the label and the notional PK.
ThreatLevel classifyThreat(double bearingFromDevice, float trackDeg,
                           double distanceKm, float altitudeFt, bool haveAlt,
                           bool valid, AirframeClass airframe, bool military) {
  if (!valid) return ThreatLevel::UNKNOWN;
  if (airframe == AirframeClass::HELICOPTER) return ThreatLevel::EXTREME_THREAT;

  ThreatLevel geo = classifyThreatGeometry(bearingFromDevice, trackDeg,
                                           distanceKm, altitudeFt, haveAlt);
  if (military && geo != ThreatLevel::HIGH_THREAT) return ThreatLevel::HIGH_THREAT;
  return geo;
}

// The geometric half, on its own so the floors above stay readable.
ThreatLevel classifyThreatGeometry(double bearingFromDevice, float trackDeg,
                                   double distanceKm, float altitudeFt, bool haveAlt) {
  if (!isfinite(trackDeg) || !isfinite(distanceKm)) return ThreatLevel::UNKNOWN;

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

// ORGANIC if the matched system belongs to the theme's own branch, JOINT if
// the ladder stepped outside it. The themes are ladders, not arsenals: the
// Marine theme owns only MADIS and hands off to NASAMS and Patriot above it,
// because the Marines field no organic medium or long-range SAM. Rather than
// blank the page for most traffic, the hand-off is labelled -- which is also
// how an expeditionary site is actually defended, by joint assets layered over
// its own SHORAD.
const char* branchTag(const char* branch) {
#if DEFENSE_THEME == DEFENSE_THEME_NAVY
  const char* own = "US NAVY";
#elif DEFENSE_THEME == DEFENSE_THEME_MARINE
  const char* own = "USMC";
#else
  const char* own = "US ARMY";
#endif
  return strncmp(branch, own, strlen(own)) == 0 ? "ORGANIC" : "JOINT";
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

  float aspectFit = (threat == ThreatLevel::EXTREME_THREAT ||
                     threat == ThreatLevel::HIGH_THREAT)   ? 1.0f
                  : (threat == ThreatLevel::MED_THREAT)   ? 0.85f : 0.7f;

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
    case ThreatLevel::LOW_THREAT:     return "LOW";
    case ThreatLevel::MED_THREAT:     return "MEDIUM";
    case ThreatLevel::HIGH_THREAT:    return "HIGH";
    case ThreatLevel::EXTREME_THREAT: return "EXTREME";
    default:                          return "UNKNOWN";
  }
}

// Half-row forms for the WEAPONS table, where a column is 13 characters.
const char* airframeShort(AirframeClass a) {
  switch (a) {
    case AirframeClass::FIXED_WING: return "FIXED";
    case AirframeClass::HELICOPTER: return "HELI";
    case AirframeClass::UAV:        return "UAV";
    default:                        return "---";
  }
}
const char* envelopeShort(Envelope e) {
  switch (e) {
    case Envelope::INSIDE:    return "IN";
    case Envelope::TOO_FAR:   return "FAR";
    case Envelope::TOO_CLOSE: return "MIN";
    case Envelope::ALT_OUT:   return "ALT";
    default:                  return "---";
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

// Skull and crossbones, 16x14, for a military target. Takes the icon slot on
// RADAR and TARGET in place of the airframe glyph; the type label and banner
// still say what kind of airframe it is. XBM, LSB = leftmost pixel.
static const unsigned char SKULL_XBM[] PROGMEM = {
  0xe0, 0x07, 0xf0, 0x0f, 0xf8, 0x1f, 0x98, 0x19, 0x98, 0x19, 0xf8, 0x1f, 0x70, 0x0e, 0xe0, 0x07, 0xa0, 0x05, 0x03, 0xc0, 0x0c, 0x30, 0xf0, 0x0f, 0x0c, 0x30, 0x03, 0xc0
};

// Icon for the target: skull if military, otherwise the airframe glyph.
void drawTargetIcon(int cx, int cy) {
  if (nearestIsMilitary()) u8g2.drawXBMP(cx - 8, cy - 7, 16, 14, SKULL_XBM);
  else                     drawTypeIcon(cx, cy, nearestCategory());
}

// Icon (~16x12) for an aircraft type, centred at (cx,cy).
void drawTypeIcon(int cx, int cy, int cat) {
  switch (cat) {
    case 8:  // helicopter -- a solid silhouette, not a stick figure
      u8g2.drawHLine(cx - 8, cy - 5, 17);          // main rotor
      u8g2.drawVLine(cx - 2, cy - 4, 2);           // mast
      u8g2.drawRBox(cx - 7, cy - 2, 9, 5, 1);      // cabin
      u8g2.drawBox(cx + 2, cy - 1, 6, 2);          // tail boom
      u8g2.drawVLine(cx + 8, cy - 4, 6);           // tail rotor
      u8g2.drawHLine(cx - 7, cy + 4, 9);           // skid
      u8g2.drawPixel(cx - 5, cy + 3);              // skid struts
      u8g2.drawPixel(cx - 1, cy + 3);
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
  return identityCategory(categoryFrom(a.category, a.onGround, a.velocityMs, a.altitudeM),
                          acTypeFor(a.icao24));
}
bool isEstimatedType(const Aircraft& a) {
  if (acTypeFor(a.icao24)[0] != '\0') return false;   // resolved, not guessed
  return a.category <= 1 && !a.onGround;              // 0 and 1 both mean "no info"
}

// Best-available airframe class for the current target: resolved type code if
// the identity lookup found one, otherwise the emitter category / kinematic
// guess. Used by everything that cares what the nearest contact actually is.
AirframeClass nearestAirframe() {
  return classifyAirframeFrom(effectiveCategory(nearest), acTypeFor(nearest.icao24));
}

// Rotor / loiter / military for the nearest contact, resolved once per poll.
// Each answer costs a scan of the identity cache plus a walk of a PROGMEM type
// list, and TARGET wanted all three every frame while loop() wanted one for
// the dwell. None of the inputs change between polls, so loop() computes them
// once, after the fetch and its identity lookups have both run.
struct NearestFlags { bool rotor, loiter, mil; int cat; char type[6]; };
NearestFlags nearestFlags;

void refreshNearestFlags() {
  nearestFlags.cat    = nearest.valid ? effectiveCategory(nearest) : 0;
  strncpy(nearestFlags.type, nearest.valid ? acTypeFor(nearest.icao24) : "",
          sizeof(nearestFlags.type) - 1);
  nearestFlags.type[sizeof(nearestFlags.type) - 1] = '\0';
  nearestFlags.rotor  = nearest.valid && nearestAirframe() == AirframeClass::HELICOPTER;
  nearestFlags.loiter = nearestFlags.rotor && isLoitering(nearest.icao24);
  // Military by address block, or by a resolved type code. The nearest contact
  // is always looked up, so the type-code half is genuinely available here
  // even though it is not for most blips.
  nearestFlags.mil    = nearest.valid && isMilitary(nearest.icao24, acTypeFor(nearest.icao24));
}
int  nearestCategory()   { return nearestFlags.cat;    }   // identity applied
const char* nearestType(){ return nearestFlags.type;   }   // "" if unresolved
bool nearestIsRotor()    { return nearestFlags.rotor;  }
bool nearestLoitering()  { return nearestFlags.loiter; }
bool nearestIsMilitary() { return nearestFlags.mil;    }

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
  drawTargetIcon(82, 16);

  u8g2.setFont(u8g2_font_6x12_tr);
  char line[24];
  snprintf(line, sizeof(line), "%.1f km %s", nearest.distanceKm, compass(nearest.bearingDeg));
  u8g2.drawStr(0, 40, line);

  if (nearest.onGround) {
    u8g2.drawStr(0, 54, "on ground");
  } else if (isfinite(nearest.altitudeM)) {
    snprintf(line, sizeof(line), "%.0f m / FL%03.0f",
             nearest.altitudeM, nearest.altitudeM * 3.28084 / 100.0);
    u8g2.drawStr(0, 54, line);
  } else {
    u8g2.drawStr(0, 54, "alt --");
  }

  // heading arrow + speed on the right ("--" when OpenSky gave no track)
  if (isfinite(nearest.trackDeg)) drawArrow(110, 34, 11, nearest.trackDeg);
  else                            u8g2.drawStr(104, 38, "--");
  if (isfinite(nearest.velocityMs)) snprintf(line, sizeof(line), "%.0f", nearest.velocityMs * 3.6); // km/h
  else                              snprintf(line, sizeof(line), "--");
  u8g2.setFont(u8g2_font_4x6_tr);
  u8g2.drawStr(98, 56, line);
  u8g2.drawStr(98, 63, "km/h");

  // altitude gauge on the far-right column (0..FL400)
  const int gT = 14, gB = 50;
  u8g2.drawFrame(125, gT, 3, gB - gT);
  if (!nearest.onGround && isfinite(nearest.altitudeM)) {
    float fl = nearest.altitudeM * 3.28084f / 100.0f;
    float fr = fl / 400.0f;
    if (fr > 1) fr = 1;
    if (fr < 0) fr = 0;
    int fh = (int)((gB - gT - 2) * fr);
    u8g2.drawBox(126, gB - 1 - fh, 1, fh);
  }

  // Alert banner, inverted so it reads as a banner rather than another data
  // row. Sits in the bottom-left, clear of the km/h readout at x=98.
  //
  // One banner, chosen by precedence, because there is only room for one.
  // Military and rotorcraft combine rather than compete -- around here a
  // military contact is quite likely to be a rotorcraft (PAT UH-60s and the
  // like), and collapsing that to just "MILITARY" would throw away the more
  // specific fact. Loiter still wins the wording, since a contact holding
  // station is the interesting case, and it blinks to say so.
  // Longest string is "MIL ROTOR LOIT" at 73 px, still clear of x=98.
  {
    bool rotor  = nearestIsRotor();
    bool loiter = nearestLoitering();
    bool mil    = nearestIsMilitary();

    const char* msg = nullptr;
    if      (mil && loiter) msg = "MIL ROTOR LOIT";
    else if (mil && rotor)  msg = "MIL ROTOR";
    else if (mil)           msg = "MILITARY";
    else if (loiter)        msg = "ROTOR LOITER";
    else if (rotor)         msg = "ROTORCRAFT";

    // Military blinks too: it is the rarer event of the two and should catch
    // the eye even when the contact is not holding station.
    if (msg && (!(loiter || mil) || ((millis() / 500) & 1))) {
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

  const RouteInfo* rt = routeFor(nearest.callsign);   // nullptr until fetched
  char line[32];

  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(0, 20, nearest.callsign);

  u8g2.setFont(u8g2_font_5x7_tr);
  snprintf(line, sizeof(line), "LINE %s", rt ? rt->airline : airlineName(nearest.callsign));
  u8g2.drawStr(0, 31, line);

  if (rt && rt->haveRoute)
    snprintf(line, sizeof(line), "RTE  %s > %s", rt->dep, rt->arr);
  else
    snprintf(line, sizeof(line), "RTE  unknown");
  u8g2.drawStr(0, 41, line);

  if (rt && rt->haveArrPos && nearest.velocityMs > 20) {
    double dk  = haversineKm(nearest.lat, nearest.lon, rt->arrLat, rt->arrLon);
    int    min = (int)(dk / (nearest.velocityMs * 3.6) * 60.0);
    snprintf(line, sizeof(line), "ETA  %dh%02dm  %.0fkm", min / 60, min % 60, dk);
  } else {
    snprintf(line, sizeof(line), "ETA  --");
  }
  u8g2.drawStr(0, 51, line);

  const char* reg = acRegFor(nearest.icao24);
  const char* typ = acTypeFor(nearest.icao24);
  char hdg[4];
  if (isfinite(nearest.trackDeg)) snprintf(hdg, sizeof(hdg), "%03.0f", nearest.trackDeg);
  else                            strcpy(hdg, "---");
  if (reg[0] || typ[0])
    snprintf(line, sizeof(line), "%s %s HDG %s", reg[0] ? reg : nearest.icao24, typ, hdg);
  else
    snprintf(line, sizeof(line), "ID %s HDG %s", nearest.icao24, hdg);
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
  float sweepDeg = sweepAngleNow();
  double sw = deg2rad(sweepDeg);
  u8g2.drawLine(cx, cy, cx + (int)(sin(sw) * R), cy - (int)(cos(sw) * R));

  // blips, dead-reckoned + persistence. All float and flat -- see struct Blip.
  const float PX_PER_KM = (float)R / MAX_KM;
  int  tBx = 0, tBy = 0;
  bool haveTarget = false;
  for (uint8_t i = 0; i < blipCount; i++) {
    float x    = blips[i].x + blips[i].vx * elapsed;
    float y    = blips[i].y + blips[i].vy * elapsed;
    float dist = sqrtf(x * x + y * y);
    if (dist > MAX_KM) continue;
    int bx = cx + (int)lroundf(x * PX_PER_KM);
    int by = cy - (int)lroundf(y * PX_PER_KM);
    if ((int8_t)i == targetBlip) { tBx = bx; tBy = by; haveTarget = true; }
    float brg = (float)rad2deg(atan2(x, y));
    if (brg < 0.0f) brg += 360.0f;

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
      // The crossing geometry, range gate and chirp queue are hardware-verified;
      // only the rotor-only path itself still awaits a live helicopter.
      if (dist <= BUZZER_RANGE_KM && sweptPast(prevSweepDeg, sweepDeg, brg))
        buzzerSweepBlip();
#endif
    } else {
      float behind = fmodf(sweepDeg - brg + 360.0f, 360.0f);
      if (behind < 50) u8g2.drawDisc(bx, by, 1);   // freshly swept
      else             u8g2.drawPixel(bx, by);     // fading
    }
  }

  prevSweepDeg = sweepDeg;

  // ring the target -- the same contact the side panel and TARGET describe,
  // which since the priority change is not necessarily the closest dot
  if (haveTarget) {
    u8g2.drawCircle(tBx, tBy, 3);
    u8g2.drawDisc(tBx, tBy, 1);
  }

  // side info panel
  const int px = 62;
  if (!nearest.valid) {
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(px, 32, "NO CONTACT");
    return;
  }

  int ec = nearestCategory();
  drawTargetIcon(px + 7, 18);
  u8g2.setFont(u8g2_font_5x7_tr);
  // A resolved type code is the most specific thing we know, so show it
  // ("H60 HELI") rather than the class name it implies; unresolved contacts
  // keep the guessed class with its '~'.
  char tname[14];
  const char* typ = nearestType();
  if (typ[0]) snprintf(tname, sizeof(tname), "%s%s", typ, isRotor(ec) ? " HELI" : "");
  else        snprintf(tname, sizeof(tname), "%s%s", isEstimatedType(nearest) ? "~" : "", typeName(ec));
  u8g2.drawStr(px + 18, 20, tname);
  // Military callsign is drawn inverted, the same treatment as the TARGET
  // banner, so the flag is visible on the page that is up most of the time.
  if (nearestIsMilitary()) {
    int w = strlen(nearest.callsign) * 5 + 3;
    u8g2.drawBox(px - 1, 25, w, 9);
    u8g2.setDrawColor(0);
    u8g2.drawStr(px + 1, 32, nearest.callsign);
    u8g2.setDrawColor(1);
  } else {
    u8g2.drawStr(px, 32, nearest.callsign);
  }

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

// WEAPONS SYSTEM. One static page of data, laid out as a table in 5x7 so it
// reads at a glance: system block on top, then three rows of label/value
// pairs in two columns. It went through a chart-plus-sidebar version and came
// back -- the graphic was fighting the text for a 128x64 panel and losing.
void screenWeapons() {
  drawHeader("WEAPONS");

  // Stale data must not carry the previous contact's solution forward.
  bool fresh = nearest.valid && lastDataMs != 0 &&
               (uint32_t)(millis() - lastDataMs) < TRACK_STALE_MS;

  int   cat     = fresh ? nearestCategory() : 0;
  AirframeClass af = fresh ? classifyAirframeFrom(cat, acTypeFor(nearest.icao24))
                           : AirframeClass::UNKNOWN;

  bool  haveAlt = fresh && !nearest.onGround && isfinite(nearest.altitudeM);
  float altFt   = haveAlt ? nearest.altitudeM * 3.28084f : NAN;
  AltitudeBand band = classifyAltitude(altFt, haveAlt);

  ThreatLevel threat = classifyThreat(nearest.bearingDeg, nearest.trackDeg,
                                      nearest.distanceKm, altFt, haveAlt, fresh,
                                      af, fresh && nearestIsMilitary());

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

  char line[28], des[12], nam[16], brn[14], rol[16], sol[10];
  copyPgm(des, sizeof(des), w.designation);
  copyPgm(nam, sizeof(nam), w.name);
  copyPgm(brn, sizeof(brn), w.branch);
  copyPgm(rol, sizeof(rol), w.role);
  formatSolution(sol, sizeof(sol), solOk, solDeg);

  u8g2.setFont(u8g2_font_5x7_tr);
  const int C2 = 68;                       // second column

  // System block, two rows: "DES NAME" with the ORGANIC / JOINT tag inverted
  // at the right, then "ROLE . BRANCH". Designation is skipped when it is the
  // same word as the name (NASAMS, MADIS).
  if (strcmp(des, nam) == 0) snprintf(line, sizeof(line), "%s", nam);
  else                       snprintf(line, sizeof(line), "%s %s", des, nam);
  u8g2.drawStr(0, 16, line);
  {
    const char* tag = branchTag(brn);
    int tw = strlen(tag) * 5 + 3;
    u8g2.drawBox(128 - tw, 9, tw, 9);
    u8g2.setDrawColor(0);
    u8g2.drawStr(128 - tw + 2, 16, tag);
    u8g2.setDrawColor(1);
  }
  snprintf(line, sizeof(line), "%s . %s", rol, brn);
  u8g2.drawStr(0, 24, line);
  u8g2.drawHLine(0, 27, 128);

  // Four rows of label/value pairs, 9 px pitch.
  // Row 1: target | altitude band
  const char* typ = fresh ? nearestType() : "";
  if (!fresh)      snprintf(line, sizeof(line), "TGT ---");
  else if (typ[0]) snprintf(line, sizeof(line), "TGT %s%s", typ, isRotor(cat) ? " HELI" : "");
  else             snprintf(line, sizeof(line), "TGT %s", airframeShort(af));
  u8g2.drawStr(0, 36, line);
  snprintf(line, sizeof(line), "ALT %s", altBandText(band));
  u8g2.drawStr(C2, 36, line);

  // Row 2: threat (inverted when EXTREME, like the banners) | solution
  snprintf(line, sizeof(line), "THR %s", threatText(threat));
  if (threat == ThreatLevel::EXTREME_THREAT) {
    u8g2.drawBox(-1, 38, strlen(line) * 5 + 3, 9);
    u8g2.setDrawColor(0);
    u8g2.drawStr(1, 45, line);
    u8g2.setDrawColor(1);
  } else {
    u8g2.drawStr(0, 45, line);
  }
  snprintf(line, sizeof(line), "SOL %s", sol);
  u8g2.drawStr(C2, 45, line);

  // Row 3: envelope with range over max range | time of flight
  if (fresh) snprintf(line, sizeof(line), "ENV %s %d/%d", envelopeShort(env),
                      (int)(nearest.distanceKm + 0.5), w.maxRangeKm);
  else       snprintf(line, sizeof(line), "ENV --- -/%d", w.maxRangeKm);
  u8g2.drawStr(0, 54, line);
  if (tofOk) snprintf(line, sizeof(line), "TOF %ds", (int)(tof + 0.5f));
  else       snprintf(line, sizeof(line), "TOF ---");
  u8g2.drawStr(C2, 54, line);

  // Row 4: PK, labelled notional -- see calcPk() for why it must be.
  if (pkOk) snprintf(line, sizeof(line), "PK %.2f NOTIONAL", pk);
  else      snprintf(line, sizeof(line), "PK --- NOTIONAL");
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
    Serial.printf("[oled] %s found at 0x%02X on SDA=GPIO%d SCL=GPIO%d\n",
                  DISPLAY_SSD1309 ? "SSD1309" : "SSD1306",
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
    // Nothing services the buzzer for the seconds the fetches take, so a tone
    // that is on right now would stay on for all of them. Park it first.
    buzzerPause();
    if (WiFi.status() != WL_CONNECTED) connectWiFi();
    fetchAircraft();
    // Route is keyed by callsign and cached; a hit costs nothing, and a GA
    // callsign never leaves the device (see fetchRoute).
    if (nearest.valid && !routeFor(nearest.callsign)) fetchRoute(nearest.callsign);
    // Identity is keyed by airframe, not callsign. The nearest contact is
    // always resolved first and unconditionally -- it drives TARGET, INTEL and
    // WEAPONS, so it must never lose out to a queued candidate. fetchAircraftInfo
    // is a no-op on a cache hit, including a cached negative, so an aircraft
    // that cycles back into nearest costs nothing.
    if (nearest.valid) fetchAircraftInfo(nearest.icao24);

    // Then any plausible-rotorcraft candidates queued during the parse, nearest
    // first, bounded by AC_LOOKUP_MAX_PER_POLL. Deliberately after the fetch:
    // each lookup builds its own TLS client and must not overlap OpenSky's.
    acDrainQueue();
    // Now that identity is as resolved as it is going to get this poll.
    refreshNearestFlags();
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
    // Coming back to the radar: the sweep-crossing test compares against the
    // angle from the *previous radar frame*, which is now a minute old.
    if (screen == SCR_RADAR) prevSweepDeg = sweepAngleNow();
  }

  render();
  buzzerService();   // non-blocking: emits any queued chirp that is now due
  delay(33);   // ~30 fps: smooth radar sweep + fast-ticking clock
}
