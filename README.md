# RP2040 SimHub steering wheel — 10 discrete LEDs + 2 encoders + 4 buttons

Firmware for a custom sim-racing wheel on a single RP2040 (Pi Pico or compatible)
over USB: **10 separate single-colour LEDs** (rev/shift lights), **2 incremental
quadrature encoders (A/B, no push switches)**, **4 physical buttons**.

Files:

- `rp2040_simhub_wheel.ino` — complete firmware (Arduino-Pico core)
- `test_wheel.py` — logic self-test + hardware test over USB serial

## 1. Investigation: how this integrates with SimHub (read first)

Existing SimHub Arduino tooling was evaluated before writing anything:

- **Legacy Arduino Setup wizard** (the one that generates sketches for Uno/Mega/
  Leonardo/Micro, with WS2812B/button/encoder options): **RP2040 is not
  supported**, and its gamepad/HID path only works on ATmega32U4 boards. It
  cannot be reused as-is. Adapting it would mean porting its whole AVR
  protocol — more work and less documented than the option below.
- **SimHub Standard Serial protocol (`SIMHUB_1.0`)**: documented binary serial
  protocol (`proto` / `ledsc` / `sleds` frames, `FF…` header, `FF FE FD`
  terminator, 115200 baud). Built for exactly this use case (PC drives LED
  hardware over CDC serial; colours/thresholds configured in SimHub). The
  protocol description ships for porting (`protocol.h` in the firmware-builder
  export). **This is what the firmware implements** — a subset (LED frames
  only), so no new protocol was invented.
- **Standard Serial Firmware Builder / device definitions**: the current
  SimHub path for commercial devices (auto-detect via USB name/VID:PID,
  LED/matrix/fan channels, vendor messages). Our firmware speaks the same
  wire protocol, so it works with SimHub's LED tooling; a full
  auto-detect device definition (custom VID:PID + USB name) is optional
  packaging, not required for function.
- **Custom Serial Devices plugin** (user-defined NCalc/JS ASCII messages):
  fallback if you ever want a text protocol, but it cannot conveniently emit
  the binary `sleds` frame — so Standard Serial is the primary path.
- **Buttons/encoders into games**: SimHub's Standard Serial channel is
  **LED-output only**; stock SimHub does not ingest button/encoder state over
  that serial link. The legacy wizard's bidirectional protocol is AVR-only.
  Therefore inputs are exposed as a **USB HID gamepad** (composite CDC+HID on
  the RP2040, which unlike an Uno can do both at once). Games see the wheel
  directly; SimHub can consume HID via its Control Mapper/joystick inputs.

Bottom line: **no existing Arduino firmware can be flashed unchanged** —
custom RP2040 firmware was required — but it speaks an **existing SimHub
protocol**, so no matching PC-side driver or SimHub plugin is required for
the headline features.

## 2. GPIO pin assignments (RP2040 GP numbers)

| Function | GPIO | Wiring | Notes |
|---|---|---|---|
| LED 1..10 | GP6..GP15 (in order) | GPIO → resistor → LED anode, cathode → GND | Active-HIGH. LED 1 = leftmost from driver view |
| ENC1 A / B | GP2 / GP3 | Encoder common → GND | `INPUT_PULLUP`, quadrature, no push switch |
| ENC2 A / B | GP4 / GP5 | Encoder common → GND | `INPUT_PULLUP`, quadrature, no push switch |
| BTN1..BTN4 | GP18, GP19, GP20, GP21 | Button → GND | `INPUT_PULLUP`, active-low, 20 ms debounce |
| Status | GP25 (onboard LED) | — | Heartbeat blink |
| Free | GP0/GP1 (UART), GP16/17/22/26/27/28 | — | Expansion; avoid GP23/24 |

Resistors (direct drive, ~3–6 mA per LED, safe for RP2040 GPIO): red ≈ 330 Ω,
green/yellow/blue/white ≈ 220 Ω (tune for even brightness). For
high-brightness LEDs, drive via NPN transistors or a ULN2803 array instead —
firmware is unchanged (still active-HIGH). One resistor per LED, common GND
with the Pico.

## 3. What works directly with SimHub vs what needs custom software

**Works directly with stock SimHub (no extra PC software):**

- All 10 LEDs as rev/shift lights via the Standard Serial `sleds` channel.
  RPM points, per-LED thresholds, and shift-flash are configured in SimHub;
  the firmware just renders frames. Colours collapse to on/off (see §5).
- Buttons/encoders as game inputs via USB HID (games + Windows controller
  panel; SimHub Control Mapper can also map them).

**Requires custom integration (extra PC-side software):**

- Reading button/encoder events as **serial text** (`BTN n PRESSED/RELEASED`,
  `ENC n +1/-1`, enabled with `events1`). Stock SimHub does not parse these;
  they exist for your own app/scripts (e.g. Python reading the COM port).
- Auto-detect device-definition packaging (custom VID:PID + `SHd,` USB name +
  picture URL). Functional LEDs work without it; it only makes the device
  self-identifying in SimHub's scanner.

**RP2040 USB constraint accounted for:** CDC (SimHub LEDs + events) and HID
gamepad run as one composite device (Pico-SDK USB stack). If HID is disabled
(`ENABLE_HID 0`), LEDs + serial still work. HID gamepad on RP2040 is generic
(no driver install).

## 4. SimHub configuration

Primary path — drive the LEDs from SimHub's standard serial LED output:

1. Flash the firmware (§6), plug in USB, note the COM port (`COMx` /
   `/dev/ttyACM0`).
2. In SimHub, add a standard-serial LED device pointed at that COM port
   (115200 baud), 10 LEDs.
3. Map RPM: rev-lights effect across LEDs 1–10 left-to-right, set per-LED
   thresholds and shift flash to taste. Because the hardware is single-colour,
   pick thresholds/brightness, not colours — any non-black colour from SimHub
   lights the LED (threshold `LED_ON_THRESHOLD` in the `.ino`).
4. Buttons/encoders: bind in-game as gamepad buttons 1–8
   (1–4 buttons; 5/6 ENC1 CCW/CW; 7/8 ENC2 CCW/CW), or via SimHub Control
   Mapper.

Fallback path — Custom Serial Devices plugin with a text protocol — is not
recommended here (it can't emit binary `sleds` cleanly); keep Standard Serial.

## 5. Behaviour notes (discrete-LED consequences)

- Each `sleds` slot's RGB is reduced with `max(R,G,B) >= 24 → ON`. SimHub
  "off/dim" states stay off; anything lit shows on. Shift flashing works
  because SimHub alternates the frames.
- Frame validation: wrong length or bad `FF FE FD` terminator discards the
  frame; partial frames time out after 100 ms. Default holds the last frame
  when SimHub is silent; set `LED_TIMEOUT_MS` (e.g. 5000) to blank instead.
- Startup self-test (chase → all-on → all-off) always runs at power-on so
  wiring/order can be checked without SimHub.
- Encoders: full 4-transition quadrature, per-detent HID pulse (80 ms),
  ~1.5 ms edge debounce. Buttons: 20 ms debounce, level HID + edge events.

## 6. Build and flashing

Arduino IDE (recommended):

1. Install the **Earle Philhower `arduino-pico`** core (Boards Manager).
2. Select board: *Raspberry Pi Pico* (or your RP2040 board).
3. `Tools → USB Stack: Pico SDK` (NOT Adafruit TinyUSB — the sketch uses the
   bundled `Joystick` HID available under Pico SDK).
4. Open `rp2040_simhub_wheel.ino`, click Upload (hold BOOTSEL + plug USB if
   the board has no sketch yet; it mounts as a UF2 drive, then upload).
5. No libraries to install (no NeoPixel dependency in this version).

PlatformIO alternative: `platform = raspberrypi`, `framework = arduino`,
`board_build.core = earlephilhower`, same `.ino` as `src/main.cpp` entry.

## 7. Test procedure

Without hardware (verifies protocol logic):

```sh
python3 test_wheel.py
```

With the flashed board (needs `pip install pyserial`):

```sh
python3 test_wheel.py COM5            # Windows
python3 test_wheel.py /dev/ttyACM0    # Linux / macOS
```

What it checks:

1. `proto` → `SIMHUB_1.0`, `ledsc` → `10`.
2. **All 10 LEDs**: chase 1→10 in order (verifies each LED + wiring order),
   then all-on, then all-off.
3. **Both encoder directions**: after `events1`, turn ENC1 CW/CCW and ENC2
   CW/CCW; expect `ENC 1 +1/-1`, `ENC 2 +1/-1`. HID buttons 5/6/7/8 pulse.
4. **All four buttons**: press/release each; expect `BTN n PRESSED/RELEASED`.
   HID buttons 1–4 follow level. Windows: also check *Game Controllers* panel.

Manual fallback (no Python): open the COM port at 115200 in a serial monitor,
power-cycle the board (self-test chase should run), send `ping` → `pong`,
send `test` → chase re-runs + `TEST DONE`.
