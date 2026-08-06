# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

ESP8266 firmware driving a 0.96" SSD1306 OLED as a desk "radar" for ADS-B
aircraft. It polls OpenSky for aircraft near a fixed home coordinate, picks the
closest one, and cycles five screens. Weather, airline/route lookup and an NTP
clock are secondary data sources. Everything is one Arduino sketch; the repo has
no test suite and no host build (but see *Testing logic on the host* below).

## Commands

PlatformIO is the primary toolchain (`pio` at `~/.platformio/penv/bin/pio`).
All commands run from `firmware/`, or use `-d firmware` from the repo root.

```bash
pio run                                        # compile
pio run -t upload                              # compile + flash (auto-detect port)
pio run -t upload --upload-port /dev/cu.usbserial-110
pio device list                                # find the board (CH340 = VID:PID 1A86:7523)
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
`[wifi]`, `[auth]`, `[fetch]`, `[wx]` and, when a helicopter is around,
`[heli]` and `[route]`.

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

`firmware/plane_spotter/plane_spotter.ino` is the whole program (~1460 lines),
organized top-to-bottom as: display constructor → data structs → math helpers →
rotorcraft tracking → network fetchers → buzzer → drawing primitives → five
`screenX()` functions → `render()` → `setup()`/`loop()`.

`loop()` is a cooperative scheduler on `millis()` deltas, no RTOS or timers. It
runs at ~30 fps (`delay(33)`) so the radar sweep animates and the clock ticks,
while network fetches happen on much slower independent intervals
(`UPDATE_INTERVAL_MS` 60 s for aircraft, `WEATHER_INTERVAL_MS` 10 min). Screen
rotation is driven by a per-screen dwell table, `SCREEN_SWAP_MS[]`, not a
uniform interval.

Rendering is full-frame: `render()` clears a full 1 KB U8g2 buffer, dispatches on
the `screen` index, and sends the whole buffer. All drawing goes through the
bus-agnostic U8g2 API, so the SPI/I²C choice touches only the constructor.

Two pieces of state decouple the fast render loop from the slow fetch loop:
`nearest` (the single closest `Aircraft`, fully populated) and `blips[]` (up to
`MAX_BLIPS` 20 lightweight lat/lon/track/speed records). Because fetches are 60 s
apart but the radar redraws 30×/s, `screenRadar()` dead-reckons every blip
forward from `lastDataMs` via `projectLatLon()` — blips visibly creep between
fetches. Anything added to the radar needs the same treatment or it will look
frozen next to the moving blips.

### Memory constraints

The ESP8266 has ~40 KB usable heap and several non-obvious rules exist purely to
stay inside it. These are load-bearing; the comments at each site explain why:

- **Do not shrink the TLS RX buffer.** `client.setBufferSizes(16384, 512)` in
  `fetchAircraft()` must stay at 16 KB — OpenSky does not negotiate MFLN, so a
  smaller buffer fails the handshake and every fetch silently returns "no
  aircraft" rather than erroring.
- **The OAuth token is refreshed *before* the data client is constructed**, so
  two 16 KB TLS buffers never coexist.
- **Use `getString()`, not `getStream()`,** for OpenSky. The response is
  `Transfer-Encoding: chunked`; streaming hands raw hex length markers to the
  JSON parser and yields nothing.
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

`DISPLAY_I2C` (in the sketch) selects the bus: `1` = 4-pin I²C panel (default),
`0` = the original 7-pin SPI panel using `PIN_OLED_CS/DC/RST` from `config.h`.
Both paths compile; check both before touching the display block.

The panel has no reset line, so reset is `U8X8_PIN_NONE`. `setup()` probes 0x3C
and 0x3D and calls `setI2CAddress(addr << 1)` — **U8g2 takes the 8-bit address**,
so 0x3C→0x78 and 0x3D→0x7A. Do not "fix" a 7-bit address into that call.

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

`LOITER_MIN_MS` is effectively a **count of polls**, not a duration — detection
is sampled at `UPDATE_INTERVAL_MS`. Two samples is the floor. One sample decides
off a single displacement, and `categoryFrom()` guesses rotorcraft for anything
under 120 km/h when OpenSky omits the category, so slow traffic would latch
immediately. Detecting faster means changing the input (poll harder, or
discriminate on track swing rather than displacement), not lowering the number.

### Buzzer

A passive buzzer on `PIN_BUZZER` (D6/GPIO12) chirps for rotorcraft only. Three
voices: a tick as the radar sweep crosses a contact, a two-tone on acquisition,
a lower insistent triple on loiter latch. All gated on `BUZZER_RANGE_KM`, and
suppressed during quiet hours — which fall *open* (audible) until NTP syncs, so
a clock that never sets cannot silence it.

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

### Type icons

OpenSky's emitter category (state index 17, needs `extended=1`) is usually `0` in
practice, so `effectiveCategory()` estimates the aircraft type from altitude and
speed and marks the guess with a leading `~` (e.g. `~Small`). Real categories,
when present, are used unmodified.

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

Upstream **PR #1** (`velezf:fix/arduinojson-v7-pin`) is open and awaiting the
owner's approval. The branch exists on `fork` only — deleting it there would
auto-close the PR. Its content is already superseded on `main` (commit 162a116
pins ArduinoJson ^7.0.4), so do not merge it into `main`.

## Unverified on hardware

Two things are written, merged and building, but have never actually run:

- **Rotorcraft rendering** (cross marker, TARGET banner, doubled dwell) — the
  loiter state machine is host-tested, but nothing has been seen on the panel,
  because it needs a helicopter in range. Watch for `[heli] new contact` and
  `[heli] … loitering` on serial.
- **The buzzer, entirely** — no buzzer was wired when it was written, and the
  board has not been flashed with it. The polarity assumption is the thing most
  likely to be wrong: if it drones continuously from power-up, flip
  `BUZZER_ACTIVE_LOW` to 0 and reflash. Boot logs
  `[buzz] enabled on GPIO12 (active-low), …`.

The flashed firmware on the board is therefore *older* than `main` — it predates
the buzzer commits.
