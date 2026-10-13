#!/usr/bin/env python3
"""Test helper for the RP2040 SimHub wheel (discrete-LED version).

Verifies the SIMHUB_1.0 subset without needing SimHub or hardware:
  1. Unit-tests the frame builder/parser logic mirror (no serial needed).
  2. Optionally talks to a flashed board over USB CDC serial.

Usage:
  python3 test_wheel.py                  # logic self-test only
  python3 test_wheel.py COM5             # full hardware test on Windows
  python3 test_wheel.py /dev/ttyACM0     # full hardware test on Linux/macOS

Hardware test sequence (discrete LEDs):
  - proto  -> expect SIMHUB_1.0
  - ledsc  -> expect 10
  - chase single LEDs 1..10, then all-on, then all-off via sleds frames
  - prompt: press each button / turn each encoder (with events1 enabled)

Requires: pyserial (pip install pyserial) only for the hardware path.
"""
import sys
import time

NUM_LEDS = 10
HDR = b"\xFF" * 6
TERM = b"\xFF\xFE\xFD"


def build_sleds(states):
    """states: list of 10 bools -> full sleds frame bytes (ON=white, OFF=black)."""
    assert len(states) == NUM_LEDS
    payload = b"".join(b"\xFF\xFF\xFF" if s else b"\x00\x00\x00" for s in states)
    return HDR + b"sleds" + payload + TERM


def threshold(rgb, thr=24):
    """Mirror of firmware latch: max(R,G,B) >= thr => ON."""
    r, g, b = rgb
    return max(r, g, b) >= thr


def logic_selftest():
    # 1. threshold mapping: SimHub colors collapse to on/off
    assert threshold((0, 0, 0)) is False
    assert threshold((10, 10, 10)) is False
    assert threshold((255, 0, 0)) is True   # red rev zone -> ON
    assert threshold((0, 255, 0)) is True   # green -> ON
    assert threshold((0, 0, 60)) is True
    # 2. frame sizes
    chase = build_sleds([True] + [False] * 9)
    assert len(chase) == 6 + 5 + NUM_LEDS * 3 + 3 == 44, len(chase)
    assert chase.startswith(HDR + b"sleds") and chase.endswith(TERM)
    # 3. each chase position lights exactly one LED
    for i in range(NUM_LEDS):
        states = [j == i for j in range(NUM_LEDS)]
        frame = build_sleds(states)
        body = frame[11:-3]
        on = [k for k in range(NUM_LEDS) if body[k*3:k*3+3] == b"\xFF\xFF\xFF"]
        assert on == [i], (i, on)
    # 4. proto/ledsc command shapes
    assert HDR + b"proto" == b"\xFF" * 6 + b"proto"
    assert HDR + b"ledsc" == b"\xFF" * 6 + b"ledsc"
    print("logic self-test: OK (threshold, framing, chase positions)")


def hardware_test(port):
    import serial
    ser = serial.Serial(port, 115200, timeout=1.0)
    time.sleep(2.0)  # allow USB CDC enumerate; startup self-test runs first
    ser.reset_input_buffer()

    def cmd(raw, expect=None):
        ser.write(raw)
        ser.flush()
        line = ser.readline().decode("ascii", "replace").strip()
        print(f"  sent {raw[6:]!r} -> got {line!r}")
        if expect is not None:
            assert line == expect, f"expected {expect!r}, got {line!r}"
        return line

    print("== proto/ledsc ==")
    cmd(HDR + b"proto", "SIMHUB_1.0")
    cmd(HDR + b"ledsc", str(NUM_LEDS))

    print("== LED chase 1..10 (watch each discrete LED light in order) ==")
    for i in range(NUM_LEDS):
        ser.write(build_sleds([j == i for j in range(NUM_LEDS)]))
        time.sleep(0.25)
    print("== all ON ==")
    ser.write(build_sleds([True] * NUM_LEDS))
    time.sleep(1.0)
    print("== all OFF ==")
    ser.write(build_sleds([False] * NUM_LEDS))

    print("== inputs: enabling events, now press BTN1..4, turn ENC1/ENC2 both ways ==")
    ser.write(b"events1\n")
    print("  (15 s capture, Ctrl-C to skip)")
    deadline = time.time() + 15
    seen = set()
    try:
        while time.time() < deadline:
            line = ser.readline().decode("ascii", "replace").strip()
            if line:
                print("  <", line)
                seen.add(line.split()[0] + line.split()[1] if line.startswith(("BTN", "ENC")) else line)
    except KeyboardInterrupt:
        pass
    ser.write(b"events0\n")
    print("captured event prefixes:", sorted(seen) or "(none — check wiring/HID)")
    print("HARDWARE TEST DONE. Also verify in OS game-controller panel:")
    print("  buttons 1-4 = BTN1-4; 5/6 = ENC1 CCW/CW; 7/8 = ENC2 CCW/CW.")


if __name__ == "__main__":
    logic_selftest()
    if len(sys.argv) > 1:
        hardware_test(sys.argv[1])
    else:
        print("(no serial port given — logic test only)")
