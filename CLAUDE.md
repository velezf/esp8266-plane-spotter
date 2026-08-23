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
TARGET / INTEL / WEAPONS are progressively deeper views of the *same* nearest
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
`nearest` (the single closest `Aircraft`, fully populated) and `blips[]` (up to
`MAX_BLIPS` 20 lightweight lat/lon/track/speed records). Because fetches are 30 s
apart but the radar redraws 30×/s, `screenRadar()` dead-reckons every blip
forward from `lastDataMs` via `projectLatLon()` — blips visibly creep between
fetches. Anything added to the radar needs the same treatment or it will look
frozen next to the moving blips.

Both are rebuilt from scratch every poll, which is why neither can carry
identity. Two tables deliberately *do* survive across polls and are keyed by
icao24: `helis[]` (loiter anchors) and `acCache[]` (resolved registrations and
type codes). If you need something to persist between fetches, it belongs in one
of those, not in `blips[]`.

### Memory constraints

Current footprint: **47.2% static RAM, 45.4% flash** (`pio run` reports both).
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
it is rebuilt from scratch every poll. `helis[]` (`MAX_HELI` 4, keyed by icao24)
holds an anchor position per airframe: stay within `LOITER_RADIUS_KM` for
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

A passive buzzer on `PIN_BUZZER` (D6/GPIO12). **Four** voices, for rotorcraft
and military contacts:

| Voice | Pattern | Fires on |
|---|---|---|
| `buzzerSweepBlip` | 1 × 4000 Hz, 25 ms | sweep crossing a rotorcraft |
| `buzzerMilitary`  | 4 × 4500 Hz, 40 ms | military contact arriving |
| `buzzerAcquire`   | 2 × 3000 Hz, 60 ms | rotorcraft acquisition |
| `buzzerLoiter`    | 3 × 2200 Hz, 120 ms | loiter latch |

They are separated on both axes a single piezo can express — pitch and rhythm.
Read down the table: pitch falls as pulses get longer. Military sits at the top
deliberately, fastest and highest, a trill rather than a beat, so it does not
read as "more of the rotorcraft alert". All four are gated on `BUZZER_RANGE_KM`
and suppressed during quiet hours — which fall *open* (audible) until NTP syncs,
so a clock that never sets cannot silence it.

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
30 fps sweep. A pattern in flight is not preempted, so a sweep tick cannot stomp
the tail of a loiter alert.

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

### Threat gating

`classifyThreat()` scores aspect (how directly the contact tracks over the
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
`AUTH:HOLD` is unconditional, and nothing is connected to anything. Envelope and
time-of-flight use published reference figures; **PK is an invented geometric
heuristic** labelled `NOTNL`, because no public data supports a real one. No
no-escape-zone or doctrinal engagement data is represented, for the same reason.
`SOLUTION` is a bearing delta over `TRACK_LOOKAHEAD_SECONDS`, not a firing
solution.

Two traps live here. **`LOW` and `HIGH` are Arduino macros** — the preprocessor
rewrites them even inside an `enum class`, so `AltitudeBand::LOW` silently
became `AltitudeBand::0`; the enumerators are `LOW_ALT`/`HIGH_ALT` for that
reason alone. And the Arduino builder emits prototypes *above* the sketch body,
so any type used in a signature must be declared near the top of the file, which
is why the enums and `WeaponSystemRecord` sit up with the data model.

Rows are 4x6 (32 columns). When editing the layout, regenerate the line-width
audit rather than eyeballing it — the longest current line is 30 columns.

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
false.

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

Almost everything is verified on real hardware now. Settled — do not
re-litigate these:

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

**The one thing still unproven is the loiter latch *geometry*** in
`trackRotorcraft()`: that a contact stayed inside `LOITER_RADIUS_KM` for
`LOITER_MIN_MS`. It is host-tested, and everything downstream of it is now
proven on hardware, but the decision itself needs a real helicopter holding
station.

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
