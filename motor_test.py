#!/usr/bin/env python3
"""Swing the PAN servo back and forth so we can SEE if wiring + power work.
Sends P (pan) between 60 and 120 deg, tilt held at 90. ~45s then stops."""
import serial, time

PORT = "/dev/cu.usbserial-0001"
ser = serial.Serial(PORT, 115200, timeout=1)
time.sleep(2.5)  # let the chip reboot after the port opens

def send(p, t=90):
    ser.write(f"P{p:03d} T{t:03d}\n".encode())
    print(f"sent P{p:03d} T{t:03d}", flush=True)

t0 = time.time()
while time.time() - t0 < 45:
    send(60); time.sleep(1.5)
    send(120); time.sleep(1.5)
send(90)
ser.close()
print("done")
