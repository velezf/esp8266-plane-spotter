# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

ESP8266 firmware driving a 0.96" SSD1306 OLED as a desk "radar" for ADS-B
aircraft. It polls OpenSky for aircraft near a fixed home coordinate, picks the
closest one, and cycles five screens. Weather, airline/route lookup and an NTP
clock are secondary data sources. Everything is one Arduino sketch — there is no
test suite and no host-side build.

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
talking at 74880 baud, not a fault. A healthy boot logs `[oled]`, `[wifi]`,
`[auth]`, `[fetch]`, `[route]`, `[wx]` lines.

The Arduino IDE also works (`firmware/plane_spotter/plane_spotter.ino`) with
U8g2 + ArduinoJson installed by hand; PlatformIO pins those in
`firmware/platformio.ini`.

## Architecture

`firmware/plane_spotter/plane_spotter.ino` is the whole program (~1170 lines),
organized top-to-bottom as: display constructor → data structs → math helpers →
network fetchers → drawing primitives → five `screenX()` functions → `render()`
→ `setup()`/`loop()`.

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

### Type icons

OpenSky's emitter category (state index 17, needs `extended=1`) is usually `0` in
practice, so `effectiveCategory()` estimates the aircraft type from altitude and
speed and marks the guess with a leading `~` (e.g. `~Small`). Real categories,
when present, are used unmodified.

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
