# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

ESP8266 firmware driving a 0.96" SSD1306 OLED as a desk "radar" for ADS-B
aircraft. It polls OpenSky for aircraft near a fixed home coordinate, picks the
closest one, and cycles six screens. Weather, airline/route lookup and an NTP
clock are secondary data sources. Everything is one Arduino sketch; the repo has
no test suite and no host build (but see *Testing logic on the host* below).

## Commands

PlatformIO is the primary toolchain (`pio` at `~/.platformio/penv/bin/pio`).
All commands run from `firmware/`, or use `-d firmware` from the repo root.

```bash
pio run                                        # compile
pio run -t upload                              # compile + flash (auto-detect port)
pio run -t upload --upload-port /dev/cu.usbserial-120
pio device list                                # find the board (CH340 = VID:PID 1A86:7523)
# NB: macOS renumbers the CH340 port on re-plug (usbserial-110 -> -120 ...).
# If upload fails with "No such file or directory", re-run device list.
pio device monitor -b 115200                   # serial, 115200
pio run -t clean
```

The xtensa toolchain PlatformIO installs for this platform is an x86_64
binary. On Apple Silicon it needs Rosetta, and every `pio run` fails at the
sketch-conversion step with `Bad CPU type in executable` without it. The fix
is `softwareupdate --install-rosetta --agree-to-license`, which is a system
change and is left to the owner. (Seen 2026-09-16 after the macOS 27 update.)

Before the first build, `cp firmware/plane_spotter/config.example.h
firmware/plane_spotter/config.h` and fill it in — `config.h` is git-ignored
(it holds WiFi credentials and the OpenSky client secret) but is `#include`d
unconditionally, so the build fails without it.

`pio device monitor` needs a real TTY and dies with `termios.error: (19,
'Operation not supported by device')` when run non-interactively. To read serial
from an agent context, drive pyserial directly instead:

```bash
~/.platformio/penv/bin/python -c "
import serial, time
s = serial.Serial('/dev/cu.usbserial-110', 115200, timeout=1)
s.setDTR(False); s.setRTS(True); time.sleep(0.1); s.setRTS(False)   # reset
end = time.time() + 30
while time.time() < end:
    d = s.read(4096)
    if d: print(d.decode('utf-8','replace'), end='', flush=True)
"
```

Expect ~200 ms of binary garbage at boot — that is the ESP8266 bootloader
talking at 74880 baud, not a fault. A healthy boot logs `[oled]`, `[buzz]`,
`[wifi]`, `[auth]`, `[fetch]`, `[wx]`, `[route]`, `[acid]` and, when a
helicopter is around, `[heli]`.

The Arduino IDE also works (`firmware/plane_spotter/plane_spotter.ino`) with
U8g2 + ArduinoJson installed by hand; PlatformIO pins those in
`firmware/platformio.ini`.

### Testing logic on the host

Most of this firmware can only be exercised with hardware and live sky, but the
pure-logic parts (loiter state machine, sweep-crossing geometry, quiet hours)
are worth testing directly, because their failure modes need a real helicopter
loitering for minutes to reproduce on-device.

The approach that works: `awk`-extract the function bodies out of the `.ino`
into `.inc` fragments, then `#include` them into a host `.cpp` that stubs
`millis()`, `Serial` and the Arduino time API. That tests shipped source rather
than a reimplementation. Watch the extraction ranges — one-liner functions like
`deg2rad` do not end in a line-initial `}`, so a naive `/start/,/^\}/` range
runs on and swallows the next function.

These harnesses live outside the repo (no test infra here) and are rebuilt as
needed; they are cheap to recreate and brittle against reordering.

## Architecture

`firmware/plane_spotter/plane_spotter.ino` is the whole program (~2300 lines),
organized top-to-bottom as: display constructor → data structs → math helpers →
type tables → identity cache/lookup → rotorcraft tracking → network fetchers →
buzzer → weapons/classification → drawing primitives → six `screenX()`
functions → `render()` → `setup()`/`loop()`.

`loop()` is a cooperative scheduler on `millis()` deltas, no RTOS or timers. It
runs at ~30 fps (`delay(33)`) so the radar sweep animates and the clock ticks,
while network fetches happen on much slower independent intervals
(`UPDATE_INTERVAL_MS` 30 s for aircraft, `WEATHER_INTERVAL_MS` 10 min). Screen
rotation is driven by a per-screen dwell table, `SCREEN_SWAP_MS[]`, not a
uniform interval.

Page order is `enum Screen`: RADAR opens with the situational picture, then
TARGET / INTEL / WEAPONS are progressively deeper views of the *same* target
contact, then the ambient pages (WX, SYSTEM). Add or reorder pages by editing
the enum and the dwell table together — `render()` and the rotorcraft
double-dwell in `loop()` both key off the names, so nothing else needs touching.
That indirection exists because the dwell rule was previously a bare
`screen == 0`, which would have silently followed the index to the wrong page on
the first reorder.

The poll rate is a budget decision, not a free knob. OpenSky charges 1 credit
per request at this bounding-box size (0.16 sq°, under the 25 sq° tier), against
~400/day anonymous or 4000/day with the OAuth2 client. 30 s is ~72% of the
registered budget; 25 s is the practical floor and 20 s exceeds it. Below ~10 s
there is nothing to gain — state vectors are served at 5 s resolution. Check
real headroom with the `x-rate-limit-remaining` response header rather than
guessing. `TRACK_STALE_MS` is pinned at ~1.5 poll intervals and must be retuned
alongside it.

Rendering is full-frame: `render()` clears a full 1 KB U8g2 buffer, dispatches on
the `screen` index, and sends the whole buffer. All drawing goes through the
bus-agnostic U8g2 API, so the SPI/I²C choice touches only the constructor.

Two pieces of state decouple the fast render loop from the slow fetch loop:
`nearest` (the target `Aircraft`, fully populated) and `blips[]` (up to
`MAX_BLIPS` 20 lightweight records).

**`nearest` is the target, not the closest contact.** Selection in
`fetchAircraft()` is by priority tier first — loitering rotorcraft, rotorcraft,
military, everything else — and by range only within the tier. The name is
historical. Plain closest-wins was handing TARGET / INTEL / WEAPONS to a 767 on
approach while a Black Hawk sat 2 km further out, which defeats the point of
the device; watched live on a PAT flight, which took TARGET at the military
tier on its first poll and held it at the rotorcraft tier once its H60 type
code came back. Two consequences: the tier uses `cat` as resolved *that* poll,
so a fast rotorcraft outranks one poll after it is first queued for identity;
and `targetBlip` carries the chosen blip's index so the radar rings the target
rather than the nearest dot. `stats.closestEver` tracks true closest
separately. A latched formation (see *Formations*) sits above all four tiers,
and within it the **lead** wins rather than the closest member: the projection
of position onto the shared velocity, with a `FORMATION_LEAD_HYST_KM` bonus for
the incumbent so two ships abreast do not trade the pages every poll.
Membership is read from the persistent tables, so it is one poll behind, the
same lag the tier already has for resolved type codes. The override applies inside `TARGET_PRIORITY_RANGE_KM` (17 km,
sketch default, overridable from `config.h`); beyond it closest wins. It began
uncapped and a departing Black Hawk held the pages from 25 km out over
airliners passing overhead, which was judged too much. Because fetches are 30 s apart but the
radar redraws 30×/s, `screenRadar()` dead-reckons every blip forward from
`lastDataMs` — blips visibly creep between fetches. Anything added to the radar
needs the same treatment or it will look frozen next to the moving blips.

**Blips are stored flat, not as lat/lon.** Each holds east/north km from home
and the velocity resolved onto the same axes (km/s), computed once at fetch
time from the range and bearing already in hand. The per-frame update is then
`x + vx*elapsed`, one `sqrt` for range and one `atan2` for bearing, and the
pixel position is a scale of x/y with no trig at all. It used to re-run the
great-circle projection, haversine and bearing per blip per frame — ~26
software-double transcendental calls each, ~400 per frame in a 16-contact sky
on an 80 MHz core with no FPU — which was most of the frame budget. Put new
radar geometry in the same flat frame. And keep the trig **double**: the float
variants (`sinf`, `atan2f`…) drag ~1 KB of libm argument-reduction tables into
`.rodata`, which on the ESP8266 is DRAM. That was measured, not guessed.

`nearestFlags` (rotor / loiter / military for the nearest contact) is the
third piece: resolved once per poll by `refreshNearestFlags()` after the
identity lookups, because each answer costs a cache scan plus a PROGMEM
type-list walk and TARGET wanted all three every frame.

`nearest` and `blips[]` are rebuilt from scratch every poll, which is why
neither can carry identity. Two tables deliberately *do* survive across polls and are keyed by
icao24: `helis[]` (loiter anchors) and `acCache[]` (resolved registrations and
type codes). If you need something to persist between fetches, it belongs in one
of those, not in `blips[]`.

### Memory constraints

Current footprint: **47.8% static RAM, 45.7% flash** (`pio run` reports both).
Note that on this chip `.rodata` counts against RAM, so string literals and
libm tables cost DRAM — compare builds with `xtensa-lx106-elf-nm -S` on the ELF
when a change grows RAM by more than its structs explain.
The number that actually bites is not static RAM but free heap *during a fetch*:
the live 16 KB TLS RX buffer leaves only **~8 KB free and ~5.4 KB contiguous**,
measured. Both memory bugs found so far lived in exactly that window, so treat
any new allocation on the fetch path as suspect. Busiest sky observed is 16
contacts against `MAX_BLIPS` 20 — the saturated case has never been tested.

The ESP8266 has ~40 KB usable heap and several non-obvious rules exist purely to
stay inside it. These are load-bearing; the comments at each site explain why:

- **Do not shrink the TLS RX buffer.** `client.setBufferSizes(16384, 512)` in
  `fetchAircraft()` must stay at 16 KB — OpenSky does not negotiate MFLN, so a
  smaller buffer fails the handshake and every fetch silently returns "no
  aircraft" rather than erroring.
- **The OAuth token is refreshed *before* the data client is constructed**, so
  two 16 KB TLS buffers never coexist.
- **Force HTTP/1.0 on the OpenSky fetch and parse straight off the socket.**
  `https.useHTTP10(true)` before `GET()`, then `deserializeJson(doc,
  https.getStream(), Filter)` — no intermediate body buffer at all.

  This replaces earlier advice to prefer `getString()` over `getStream()`. That
  advice was right about the symptom (streaming a *chunked* body feeds raw hex
  length markers to the parser) but the cure was worse: `getString()` was
  silently truncating **~15% of polls**. Chunked means `_size` is -1, so
  `getString()`'s own `reserve()` never runs, and the core allocates a fresh
  `String` per chunk header via `readStringUntil('\n')` — all inside the ~5.4 KB
  contiguous window left by the live 16 KB TLS buffer. One failed allocation and
  bytes-written stops matching bytes-declared, `writeToStream()` returns
  `HTTPC_ERROR_STREAM_WRITE` (-10), and `getString()` — which returns
  `const String&` and has no error channel — hands back a body cut mid-token.
  The only symptom was a downstream `IncompleteInput`.

  HTTP/1.0 forbids chunked encoding, which removes the chunk headers, their
  allocations, *and* the reason streaming was unsafe. Note OpenSky sends no
  `Content-Length` even then (`getSize()` is -1; it closes to signal end) —
  that is expected and streaming does not need it.

  **Do not "restore" `getString()` here.** Pre-reserving the buffer was tried
  and made it worse — the reservation consumed the contiguous space the chunked
  decoder needed, pushing failures from ~15% to ~28%.
- **JSON is parsed through a `DeserializationOption::Filter`** that keeps only
  the ~11 state-vector indices actually used. Widening the filter or raising
  `SEARCH_RADIUS_DEG` (0.2° ≈ a 44×35 km box) increases parse RAM and can reboot
  the board.

ArduinoJson is **v7** — bare `JsonDocument d;` with no size template parameter.
v6 syntax will not compile.

### Display

Currently I²C on **non-standard pins**: `SDA = GPIO13 (D7)`, `SCL = GPIO14 (D5)`,
not the ESP8266 defaults GPIO4/GPIO5. Those are the pads the original 7-pin SPI
panel used for MOSI/SCLK, and the solder joints were reused. This works because
ESP8266 `Wire` is a bit-banged software I²C master — any GPIO pair is valid as
long as the pins reach `Wire.begin()`, which the U8g2 `HW_I2C` backend does when
the constructor is given a clock/data pair.

Two independent switches in the sketch, giving four combinations that all
compile — check the matrix when touching this block:

- `DISPLAY_I2C` — bus. `1` = I²C (default), `0` = 4-wire hardware SPI using
  `PIN_OLED_CS/DC/RST` from `config.h`.
- `DISPLAY_SSD1309` — controller. `1` = SSD1309, the 2.42" panel (**the current
  hardware**); `0` = SSD1306, the 0.96" panel.

Both panels are 128×64, so **every layout is identical between them**. The 2.42"
is the same pixel grid at ~2.5× the linear size — it buys legibility, not room.
That is also why portrait (`U8G2_R1`/`R3`) was rejected: rotating gives 16
columns at 4x6 instead of 32, which the WEAPONS page cannot fit (its longest
line is 30), and `screenRadar()` is a hard left/right split — disc at `cx=31`,
info panel at `px=62`.

**The SSD1309 needs its own init sequence; SSD1306 init is not good enough.**
This is a trap because it half-works: a panel hot-plugged into an already-running
board renders fine on SSD1306 init, then comes up wrong after the next cold
reset, which makes it look like the reflash broke it. If the `[oled]` probe finds
the panel at 0x3C but the screen is wrong, that is an init problem, not wiring.

`SSD1309_NONAME2` picks between the two init variants these modules ship with
(`0` = NONAME0, the one that works here; `1` = NONAME2). Try flipping it before
suspecting hardware.

Reset: the 0.96" has no reset line, so it always passes `U8X8_PIN_NONE`. The
2.42" breaks `RES` out even in I²C mode — `PIN_OLED_RST_I2C` in `config.h` takes
the GPIO, or `-1` for none. **`-1` is confirmed working on this hardware**; the
panel does not in practice need the pulse.

`setup()` probes 0x3C and 0x3D (both controllers use the same pair) and calls
`setI2CAddress(addr << 1)` — **U8g2 takes the 8-bit address**, so 0x3C→0x78 and
0x3D→0x7A. Do not "fix" a 7-bit address into that call.

### Rotorcraft

Helicopters are singled out across the UI: a cross marker on the radar that
skips the persistence fade, an inverted banner on TARGET, and double dwell on
that screen (`loop()` computes `dwell` locally rather than reading
`SCREEN_SWAP_MS[]` directly).

Loiter detection needs identity across fetches, which `blips[]` cannot provide —
it is rebuilt from scratch every poll. `helis[]` (`MAX_HELI` 8, keyed by icao24;
sized like `MAX_MIL` for a formation plus a couple of singles, since eviction
resets a member's formation count and re-alerts it) holds an anchor position
per airframe: stay within `LOITER_RADIUS_KM` for
`LOITER_MIN_MS` and it latches as loitering; drift outside and the anchor resets,
because that is transit rather than orbit.

**Two floors constrain `LOITER_MIN_MS`, and the geometric one binds.** The
obvious floor is sampling: it is a count of `UPDATE_INTERVAL_MS` samples, so at
least two. But the threshold must also outlast a slow *transit* crossing the
anchor radius, or it latches on aircraft merely passing through — and that is
the tighter limit. A 120 km/h contact (the fastest thing `categoryFrom()` will
guess as a rotorcraft) covers only 2.0 km in 60 s, still inside the 3 km radius,
so a 60 s threshold false-latches; at 120 s it is 4.0 km out and re-anchors.

Faster polling therefore does **not** shorten this — it only buys robustness
(120 s is 4 samples at a 30 s poll, not 2). Going genuinely faster means
shrinking `LOITER_RADIUS_KM`, which risks re-anchoring on wide orbits, or
discriminating on track swing rather than displacement.

### Buzzer

A passive buzzer on `PIN_BUZZER` (D6/GPIO12). **Five** voices, for rotorcraft,
military and formation contacts:

| Voice | Pattern | Fires on |
|---|---|---|
| `buzzerSweepBlip` | 1 × 4000 Hz, 25 ms | sweep crossing a rotorcraft or formation member |
| `buzzerMilitary`  | 4 × 4500 Hz, 40 ms | military contact arriving |
| `buzzerFormation` | 6 × 3500 Hz, 50 ms | formation latch (priority 2) |
| `buzzerAcquire`   | 2 × 3000 Hz, 60 ms | rotorcraft acquisition |
| `buzzerLoiter`    | 3 × 2200 Hz, 120 ms | loiter latch |

They are separated on both axes a single piezo can express — pitch and rhythm.
Read down the table: pitch falls as pulses get longer. Military sits at the top
deliberately, fastest and highest, a trill rather than a beat, so it does not
read as "more of the rotorcraft alert". Formation is the longest run by far and
the only voice at priority 2, so it cuts anything else. All five are gated on `BUZZER_RANGE_KM`
and suppressed during quiet hours — which fall *open* (audible) until NTP syncs,
so a clock that never sets cannot silence it.

**The three arrival voices fire on the first *in-range* poll, not the first
sighting.** `helis[]` and `milSeen[]` carry `acquireAlerted` / `loiterAlerted`
/ `alerted` flags for exactly this. The distinction matters because the search
box edge is always outside buzzer range: a 0.2° box is 17–22 km from home and
the gate is 15 km, so anything that flies in is first seen out of range. The
original code marked the airframe "announced" at first sighting and only
range-checked that one poll, which meant the military and acquisition voices
could not sound for any contact that did not pop into existence already close
— a military rotorcraft was watched transit past with its banner up and no
chime. The loiter flag resets on re-anchor so a second orbit alerts again.

**Priority, not first-come.** `buzzerChirp()` takes a priority: the sweep
tick is 0, the three arrival alerts are 1, formation is 2. A higher priority cuts a lower one that is
sounding; equal priority is first-come, so alerts never cut each other short
and a tick still cannot stomp the tail of a loiter alert. This replaced a plain
"in flight wins" rule under which a military trill arriving during a 25 ms
tick was silently dropped. `loop()` also calls `buzzerPause()` before the fetch
block, because nothing services the buzzer for the seconds the fetch and its
lookups take and a tone that was on at that instant used to stay on for all of
it. Alerts queued during the fetch play after the lookups, so the trill can
land several seconds after `[mil] new contact`; that is latency, not loss.

`prevSweepDeg` is re-seeded on the swap back to RADAR. Left stale for a
minute it landed inside `sweptPast()`'s 30° window about one return in twelve
and fired a stray tick for whatever sat in that arc.

**Polarity is the trap here.** The hardware is a 3-pin module driven by an
S9012, which is a **PNP** transistor: it conducts on a LOW base, so the module
sounds when the pin is pulled low and must idle **HIGH**. Two consequences that
are easy to get backwards:

- Parking the pin LOW at boot — the intuitive way to keep it quiet — makes this
  module sound continuously instead.
- The core's `noTone()` ends with `digitalWrite(pin, 0)`, so anything relying on
  `tone()`'s duration argument leaves the buzzer howling after every chirp.

`buzzerService()` therefore drives `tone()`/`noTone()` itself and restores
`BUZZER_IDLE_LEVEL` by hand once a chirp ends. Do not "simplify" it back to
`tone(pin, freq, duration)`. `BUZZER_ACTIVE_LOW` covers the other polarity (bare
2-pin element or NPN module).

Voices sit at 4000 / 3000 / 2200 Hz. These elements have no oscillator and want
**2–5 kHz** — below ~2 kHz they go noticeably quiet, so keep new tones in band.

Everything is non-blocking. Never add `delay()` here — it would stutter the
30 fps sweep. Preemption is by priority, above.

The pin choice is constrained, not arbitrary: GPIO16 (D0) is off the normal
GPIO mux and cannot do `tone()`; GPIO0/GPIO2 must be HIGH at boot and a buzzer
coil dragging them down prevents booting; GPIO15 must be LOW at boot. GPIO12 is
free in both display builds.

`BUZZER_ENABLE`, `BUZZER_ACTIVE_LOW` and `BUZZER_SWEEP_BLIP` are independent
compile-time switches — check the combinations still build when touching this.

### Aircraft identity lookup

OpenSky almost never populates the emitter category (`cat=0`), which is why
`categoryFrom()` guesses airframe type from speed and altitude.
`fetchAircraftInfo()` resolves an airframe by icao24 through three tiers —
hexdb.io, then adsbdb.com, then adsb.lol — giving a registration and an ICAO
type code. A type code is hard identity, so it wins over both the category and
the kinematic guess, for **every** contact now: the fetch parse applies it to
`cat` directly, which is what puts a cross on a helicopter transiting above the
120 km/h guess threshold and takes a wrong one off a slow fixed-wing. Measured
coverage over 24 aircraft overhead: 17/24, 20/24, 24/24.

**Who gets looked up, and why it is gated.** The nearest contact always is,
unconditionally and first — it drives TARGET / INTEL / WEAPONS. Beyond that,
only contacts that could plausibly *be* rotorcraft and are close enough to
matter: `AC_LOOKUP_ENVELOPE_KMH` / `_M` and `AC_LOOKUP_RANGE_KM`. Measured over
36 minutes of this airspace, 168 distinct airframes/hr pass through, 29% fall
inside the envelope, and only **3%** are also within 15 km. The range gate is
what keeps this affordable; without it the rate approaches the whole-sky figure.
Resolving a 700 km/h contact at FL350 buys nothing — it is already classified
correctly.

**The binding cost is time, not API quota.** Tier calls are synchronous HTTPS on
the same thread as the 30 fps render loop, so each one freezes the sweep and the
clock. Measured: **~1.2 s** for a tier-1 hit, **~3.5 s** when both databases miss
and it falls through to tier 3 (`[acid]` logs the duration). `AC_LOOKUP_MAX_PER_POLL`
bounds this — at 2, plus the unconditional nearest, worst case is ~10 s of frozen
display per 30 s poll. Raising it trades smoothness for coverage. The real fix
would be spreading lookups across the render loop rather than bursting them
after the fetch.

Candidates are queued during the parse and drained **after** `fetchAircraft()`
returns — never during it, because each lookup builds its own TLS client and two
16 KB RX buffers must not coexist.

**Positions never come from anywhere but OpenSky.** That is the whole point of
the split: tier 3 is a community-run service, and if it rate-limits or vanishes
the chain degrades to tier 2, then to the guess, and nothing on screen breaks.
Keep it that way — moving the *feed* to a best-effort endpoint would mean a 429
blanks the entire display. `AC_LOOKUP_TIER3` turns it off.

**Routes are cached the same way**, in `routeCache[]` (`ROUTE_CACHE_N` 8,
LRU, keyed by callsign, negatives included), and `fetchRoute()` only queries
hexdb for callsigns that look like an airline flight — three letters then a
digit. Before this, every swap of nearest cost two sequential TLS round trips
with the display frozen, including for N-numbers that can never have a route,
and those seconds were not counted in the lookup budget above.

Results live in an `AC_CACHE_N`-entry LRU table (`acCache[]`), negatives
included, so an unknown icao24 is not re-queried every poll. This was a single
record until the lookup widened, and that was costing real requests: on an
approach path aircraft cycle through faster than the six screens do, so the same
few tails were re-resolved every time they came back around. A cache hit is now
free, which is what pays for looking beyond the nearest contact — sizing note in
`config.h` is based on ~24 distinct airframes per 10 minutes here.

Tier 3 is plain HTTP, so it skips the 16 KB TLS buffer; note it is a *live*
query and only knows airborne aircraft. It can also return a registration with
an **empty type code** — that is cached as resolved and not retried, so identity
without classification is a real state to expect.

The type tables (`HELI_TYPES`, `UAV_TYPES`, `typeInList()`) sit *above* the
`#if AC_LOOKUP_ENABLE` guard on purpose: `classifyAirframeFrom()` needs them
whether or not lookups are compiled in. They were inside it, which meant
`AC_LOOKUP_ENABLE 0` did not build at all.

### Military contacts

Detected from the **icao24 address block** (`MIL_HEX`), and confirmed by ICAO
type code (`MIL_TYPES`) when one has resolved. The address has to be primary,
and the reason is structural: identity lookups are gated to the rotorcraft
envelope, so a C-130 at 400 kt is never resolved and a type-code-only detector
would never see it. The address is in every state vector already, so the check
is one integer compare and works on any contact at any speed or range.

**Only the US block is listed**, deliberately. This device sits under the
Washington DC area where Andrews traffic is the realistic case, and a wrong
range is worse than a missing one because it paints civil aircraft as military.
Other nations' allocations are published and easy to add, but none has been
checked against traffic from here.

`milSeen[]` (`MAX_MIL`) remembers announced airframes so the alert fires on
arrival, not every poll — same shape as `helis[]`, and for the same reason:
`blips[]` is rebuilt each fetch and cannot remember anything. **`MAX_MIL` is
sized for a formation, not the typical case.** Routine occupancy here is zero;
50 minutes of sampling produced no military contact at all. But transports
arrive several at a time, and overflow is not graceful — LRU eviction means an
evicted airframe is seen again next poll and re-announced, so the alert repeats
every 30 s. Verified by forcing it: 6 contacts against 4 slots double-announced
five of them. Hence 8.

On screen, military and rotorcraft **combine rather than compete** —
`MILITARY` / `MIL ROTOR` / `MIL ROTOR LOIT`. Around here a military contact is
quite likely to *be* a rotorcraft (PAT UH-60s and similar), and collapsing that
to just "MILITARY" would discard the more specific fact. There is only room for
one banner, so precedence picks it; loiter still wins the wording. Military
blinks even without loiter, being the rarer event.

### Formations

Two or more special contacts (rotorcraft or military) moving together. This is
the "worth running outside for" event, and nothing else in the firmware could
see it: every other tracker looks at one airframe at a time.

`detectFormations()` runs once per poll, after the parse loop and before the
expiry passes, over `blips[]` — the flat frame already holds every contact's
position and velocity, so a pair test is one squared distance and one squared
velocity difference. **The velocity-difference magnitude is the discriminator**,
not track angle: it tests speed and heading in one number with no wrap, and it
is what rejects a crossing pair (close, diverging) and today's spread-out trio
(same heading, kilometres apart). Candidates are rotorcraft and military blips
with known kinematics; a `vx = vy = 0` contact is excluded because two of them
would agree perfectly, the same missing-data trap as the NAN rule. Union-find
over the candidates gives cluster sizes, so a chain A–B–C is one three-ship.

Persistence is per airframe in `helis[]` / `milSeen[]` (both, for a military
rotorcraft): `formPolls` counts consecutive clustered polls and latches at
`FORMATION_MIN_POLLS`; breaking formation resets the count and the alert flag,
so a flight that splits and re-forms alerts again, like the loiter re-anchor.
Blips carry their icao24 for the poll purely so this pass can find the table
entry — it is a key, not identity that survives a fetch. Three members latching
on the same poll produce three `buzzerFormation()` calls that collapse to one
voice, which is the intended "one event, one alert".

On screen a formation shows everywhere the target does: the TARGET banner
appends the size (`ROTOR FLT x3`, `MIL FLT x3`, `ROTOR LOIT x3`; longest 73 px,
same limit as before), RADAR and INTEL draw an inverted `x3` tag beside the
callsign via `drawFormationTag()`, every member gets the radar cross and the
sweep tick — military fixed-wing included, since each ship should register on
its own — and WEAPONS reads `THR IMMINENT` (pulsing) with `TGT H60 x3`. The
join line between members was considered and skipped.

`FORMATION_SEP_KM` is loose (2 km) because OpenSky's per-aircraft position
times within one poll differ by seconds, which at rotorcraft speeds is hundreds
of metres of false spacing. **Neither threshold has been tuned on a real
pair.** The count is a lower bound: military flights often have one ship on
ADS-B and the rest dark, and nothing this module can see fixes that — it is a
job for a future project that detects non-ADS-B traffic.

### Threat gating

`classifyThreat()` is identity first, geometry second: a formation member is
**IMMINENT** unconditionally, a rotorcraft is **EXTREME** and a military
fixed-wing floors at **HIGH**; only civil fixed-wing traffic is scored by
geometry (`classifyThreatGeometry()`).
That is a product decision, not a modelling one — around here the rotorcraft
is what the device exists for, and scoring one LOW because it was tracking
away read as broken. EXTREME is drawn inverted on WEAPONS like the banners.

The geometric half scores aspect (how directly the contact tracks over the
device) against **slant** range, not ground distance — altitude is most of how
far away an aircraft is, and on ground distance alone a jet at FL350 overhead
scored the same as a Cessna at 2000 ft on the same track.

HIGH additionally needs a known altitude under `THREAT_HIGH_MAX_FT`. The slant
gate alone is not enough: 3 km admits anything below ~9800 ft when overhead, so
the ceiling is what actually enforces "low" while slant enforces "close".

`THREAT_HIGH_SLANT_KM` has a floor set by the poll rate, not by eyesight: a pass
is only guaranteed to be sampled if the contact dwells inside the bubble longer
than one poll interval. At 250 km/h a 1.5 km bubble is a 43 s dwell — missable
at 60 s polling, guaranteed at 30 s. Real naked-eye tail-number range (~0.3–0.5
km) is a 14 s dwell and would be missed on most passes at any affordable rate.
Retune this whenever `UPDATE_INTERVAL_MS` changes.

### WEAPONS SYSTEM page

A themed air-defense reference display (screen 6) layered over the same ADS-B
data. It classifies the nearest contact, picks a system from a PROGMEM table,
and shows a track-lead angle plus notional envelope figures.

Scope is deliberate and should stay that way: every contact is FRIENDLY,
firing authorization is unconditionally withheld (the `AUTH:HOLD` label was
removed from the page as constant noise; the policy stands), and nothing is
connected to anything. Envelope and
time-of-flight use published reference figures; **PK is an invented geometric
heuristic**, because no public data supports a real one. It carried a
`NOTIONAL` suffix on screen until the owner judged that implied by the page
and had it dropped; the caveat stays in the `calcPk()` comment. No
no-escape-zone or doctrinal engagement data is represented, for the same reason.
`SOLUTION` is a bearing delta over `TRACK_LOOKAHEAD_SECONDS`, not a firing
solution. It was dropped once for space and put back because it is the
thematic touch; it lives in the sidebar.

**Layout is a table, 5x7, no graphics.** Two system rows (designation and
name with the tag; role and branch), a rule, then four rows of label/value
pairs in two columns (second column at x=68): TGT | ALT, THR | SOL,
ENV | TOF, PK. Same content as the original nine-row 4x6 page, nothing
dropped but the PK suffix. It went through a
range-versus-altitude envelope chart with a 4x6 sidebar and came straight
back: on a 128x64 panel the graphic fought the text and the owner wanted
something readable from across the room, not a plot. Do not reintroduce
graphics here. Half-row values use the short forms (`airframeShort()`,
`envelopeShort()`): a column is 13 characters of 5x7.

**Themes are ladders, not arsenals.** The Marine theme owns only MADIS and
hands off to NASAMS and Patriot above it, because the Marines field no organic
medium or long-range SAM; a strict filter would blank the page for most
traffic. `branchTag()` labels the hand-off instead: **ORGANIC** when the
matched system's branch is the theme's, **JOINT** otherwise. The tag is a
prefix match on the record's `branch` string, so keep those strings starting
with the service name.

Two traps live here. **`LOW` and `HIGH` are Arduino macros** — the preprocessor
rewrites them even inside an `enum class`, so `AltitudeBand::LOW` silently
became `AltitudeBand::0`; the enumerators are `LOW_ALT`/`HIGH_ALT` for that
reason alone. And the Arduino builder emits prototypes *above* the sketch body,
so any type used in a signature must be declared near the top of the file, which
is why the enums and `WeaponSystemRecord` sit up with the data model.

Rows are 5x7 (25 columns full width, 13 per column). When editing them,
regenerate the line-width audit rather than eyeballing it — the role/branch
row is the longest at 24 (`AREA DEFENSE . US/NORWAY`), which is why
`WS_D_BRN` is `US/NORWAY`, `WS_F_BRN` is `USMC` and `WS_C_NAM` is `STRYKER`
(the name row has to leave 38 px for the `ORGANIC` tag).

### Spelled-out airframe names

A resolved ICAO type code is drawn as a name, not a code: TARGET shows
`BOEING 777-300ER` in a 5x7 row directly under the callsign, and the radar
info panel shows the bare model (`777-300ER`, `UH-60`) beside the type icon.
This exists because guests watching the panel asked to see "A380" or "777"
and the code was buried on INTEL, where it still appears raw beside the
registration.

`airframeName()` looks the code up in `TYPE_NAMES[]`, a PROGMEM table of
maker + model (~160 entries, what actually flies over the US Northeast),
then falls back to two family rules — Boeing `B7XY` → `7X7-Y00`, Airbus
`A33Y`/`A34Y` → `A330-Y00`/`A340-Y00` — and finally to the raw code. Table
entries win over the rules, which is how the irregulars (`B77W`, `B789`,
`B748`, the MAX family) get real names. Three-letter codes are stored padded
to four (`"H60 "`) so one fixed-width field serves all; the query is padded
the same way.

Width limits are structural, not stylistic: a model is at most **9** characters
because the radar panel has 48 px of 5x7, and maker + space + model is at
most **19** because the TARGET row must clear the heading arrow at x=99. The
compiler enforces the model width through the `char model[10]` field, and
the TARGET width is why the Bombardier business jets are `CL-300` rather than
`CHALLENGER 300`. When adding entries, audit the full-name width too — the
compiler only catches the model.

Records are read with `memcpy_P`; the ESP8266 faults on unaligned byte
reads from flash. The table costs ~4.5 KB of flash and no DRAM.

### Type icons

OpenSky's emitter category (state index 17, needs `extended=1`) is usually `0` in
practice, so `effectiveCategory()` estimates the aircraft type from altitude and
speed and marks the guess with a leading `~` (e.g. `~Small`). Real categories,
when present, are used unmodified.

### Missing data is NAN, never 0

OpenSky leaves velocity (9) and both altitudes (7, 13) null often enough that
this is a hot path, not an edge case. **Parse them as `NAN` when null**, which is
the sentinel the threat and weapons layer already tests for with `isfinite()` —
there are a dozen such guards, and defaulting to `0.0f` in the fetch made every
one of them dead code.

The bug this caused is the reason to keep it that way. `categoryFrom()` guesses
"helicopter" from *slow and low*, so a contact with null velocity and altitude
read as 0 km/h at 0 m and guessed rotorcraft. A poll where most vectors were
incomplete drew the entire radar as rotorcraft crosses, and — worse, because it
outlives the poll — fed airliners into `helis[]`, where they refresh their own
`lastSeenMs` every fetch, never expire, crowd out the `MAX_HELI` 4 slots, and can
latch as *loitering*. `categoryFrom()` therefore returns `0` (unknown) rather
than guessing when either input is not finite.

Anything reading these fields must handle `NAN`. The display sites in
`screenNearest()` show `alt --` / `--` km/h rather than printing `nan`; the
dead-reckoning and track-lead paths already guarded correctly, since `NAN > 0` is
false. True track (10) is `NAN` too now: as `0` it dead-reckoned the blip due
north and pointed the TARGET arrow there. TARGET draws `--` in place of the
arrow and INTEL shows `HDG ---`; a blip with unknown track or speed holds
still, which is honest.

Note the identity tiers do **not** rescue this. `classifyAirframeFrom()` prefers a
resolved ICAO type code over the guess, but `fetchAircraftInfo()` only ever runs
on the nearest contact — so the other 19 blips, and `trackRotorcraft()` at fetch
time, have nothing but the kinematic guess to go on.

## Workflow

**Ask before `git commit` or `git push`.** An instruction about *where* work
should go ("do this on a feat/ branch") is not authorization to commit it, and
approval for one commit does not carry to the next. Creating a branch and
building on it unprompted is fine; committing and pushing is not.

Feature work goes on a `feat/…` branch, merged to `main` with `--no-ff`, and the
branch is deleted both locally and on `fork` once merged.

## Git

Two remotes, and they are not interchangeable:

- `fork` → `git@github.com:velezf/esp8266-plane-spotter.git` — the user's own
  repo. `main` tracks `fork/main`; this is the normal push target.
- `origin` → `https://github.com/DaniloCannas/esp8266-plane-spotter.git` — the
  upstream project this was forked from. Do not push here.

**Upstreaming is not planned.** This fork has accumulated several fixes that are
general rather than local to this build — the null-kinematics rotorcraft
misclassification, emitter category 1 read as identity, and the chunked-response
truncation that was silently losing ~15% of polls. All three would benefit any
user of the upstream project. They are staying here anyway: upstream PR #1 has
sat unreviewed since 2026-06-30, so the maintainer is not currently maintaining
it and preparing more PRs would be effort spent on a queue nobody reads.

Do not propose upstreaming again unless the owner becomes active. If that
changes, send the truncation fix first — it is the most valuable and the most
self-contained, and the response tells you whether the rest is worth preparing.

Upstream **PR #1** (`velezf:fix/arduinojson-v7-pin`) is still open, left that
way deliberately rather than withdrawn. Its content is already superseded on
`main` (commit 162a116 pins ArduinoJson ^7.0.4), so do not merge it into `main`.
The branch exists on `fork` only and deleting it there auto-closes the PR — that
no longer matters much, but do not delete it as a side effect of tidying.

## Unverified on hardware

Everything below is verified on real hardware. Settled — do not re-litigate
these:

- **Display.** 2.42" SSD1309 on I²C at 0x3C, landscape, NONAME0 init, no reset
  line. All six screens render; the 0.96" SSD1306 remains a one-line fallback.
- **Buzzer polarity and element.** 3-pin PNP/S9012 module on D6/GPIO12;
  `BUZZER_ACTIVE_LOW 1` is correct. All four voices sounded through the real
  `buzzerChirp`/`buzzerService` path — not raw `tone()` — and were audible in
  the finished enclosure against bar-level ambient noise. The by-hand
  `BUZZER_IDLE_LEVEL` restore does prevent the `noTone()` drone.
- **The sweep-chirp trigger chain.** `sweptPast()` crossing geometry (`sweep`
  landed within a few degrees of `brg` on every tick), the `BUZZER_RANGE_KM`
  gate, the screen gate, and the chirp queue.
- **The whole rotorcraft/military display path.** Cross marker (and that it
  skips the persistence fade), doubled TARGET dwell, the pulsing loiter ring,
  and every banner including the combined `MIL ROTOR` and `MIL ROTOR LOIT`.
- **Identity lookups**, all three tiers, and a type code correcting a wrong
  kinematic guess in flight — watched live on a C172 on approach.
- **The loiter latch geometry** in `trackRotorcraft()`: a contact staying
  inside `LOITER_RADIUS_KM` for `LOITER_MIN_MS`. Host-tested first, then seen
  live on a real helicopter holding station on 2026-09-15.
- **Range-entry alert gating** (September 2026 alert/perf branch): a PAT
  flight first logged as `[mil] new contact` beyond `BUZZER_RANGE_KM` sounded
  the trill on the first poll inside it, watched live on 2026-09-15. That
  also exercised chirp priority, `buzzerPause()`, the flat-coordinate radar,
  the route cache and weather over HTTP/1.0 in ordinary running.

**Unverified: the formation feature** (September 2026 `feat/formations`).
The clustering, latch, split/re-form and range-entry logic was host-tested
against the shipped source with the awk-extract harness (nine scenarios, all
passing), but nothing downstream has been seen on hardware: the fifth voice,
the `x3` banners and tags, the cross and tick on a military fixed-wing member,
`THR IMMINENT`, the lead selection and its hysteresis, and the RAM cost (the
footprint figures above predate it; `Blip` grew by 8 bytes, `MAX_HELI` went
from 4 to 8). The build itself could not be run on the machine that wrote it
(see the Rosetta note under *Commands*). Force it the usual way: a throwaway
build that clones the nearest blip twice with a small offset and the same
velocity latches a three-ship in two polls.

**How the rest got closed, because it applies to whatever is unverified next:**
forcing the state beats waiting for it. Rather than wait weeks for a helicopter,
a throwaway build forced the nearest contact to read as a military rotorcraft
and latched loiter after three polls. That exercised rendering, dwell, banners
and voices in one sitting. Be precise about what such a test proves: it verified
everything *downstream* of the latch, not the latch itself. Earlier the same
trick — hoisting the sweep chirp out of its `isRotor()` branch — proved the
trigger chain.

Watch for `[heli] new contact`, `[heli] … loitering` and `[mil] new contact` on
serial. Note that `rotor=N` counts and `[heli]` lines from before the *Missing
data is NAN* fix cannot be trusted — misclassified jets were entering `helis[]`
— so old sightings are not evidence either way.

The per-contact `[blip]` dump that closed most of the list is now permanent,
behind `LOG_BLIP_DUMP` in `config.h` (default **0**). It logs icao24, callsign,
range, speed, altitude, resolved `cat` and the ROTOR/MIL flags — the difference
between "`rotor=0`, no idea why" and a diagnosis; it is how the C172
misclassification was spotted live. It defaults off for the same reason as
`LOG_REQUEST_URL`: each line pairs a public aircraft identity with its distance
from the device, and a few simultaneous (position, distance) pairs recover the
device coordinates by trilateration — do not paste `[blip]` logs anywhere
public. It spent two weeks as uncommitted "remove before committing" code; that
guard failed once (a marker comment reached `main`), which is why it is a
config switch now.
