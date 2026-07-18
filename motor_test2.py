#!/usr/bin/env python3
"""Move PAN, then TILT, taking turns, so we can see which motor responds.
PAN swings (tilt held), then TILT swings (pan held). ~32s."""
import serial, time

ser = serial.Serial("/dev/cu.usbserial-0001", 115200, timeout=1)
time.sleep(2.5)  # let the chip reboot after the port opens

def send(p, t):
    ser.write(f"P{p:03d} T{t:03d}\n".encode())
    print(f"sent P{p:03d} T{t:03d}", flush=True)

for i in range(4):
    print("=== PAN should move now ===", flush=True)
    send(60, 90); time.sleep(1.5)
    send(120, 90); time.sleep(1.5)
    send(90, 90); time.sleep(1.0)
    print("=== TILT should move now ===", flush=True)
    send(90, 60); time.sleep(1.5)
    send(90, 120); time.sleep(1.5)
    send(90, 90); time.sleep(1.0)
ser.close()
print("done")
