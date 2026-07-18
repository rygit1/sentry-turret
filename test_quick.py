#!/usr/bin/env python3
"""Quick pan swing to watch the goalpost move, then leave both at center."""
import serial, time
ser = serial.Serial("/dev/cu.usbserial-0001", 115200, timeout=1)
time.sleep(2.5)
def send(p, t=90):
    ser.write(f"P{p:03d} T{t:03d}\n".encode()); print(f"sent P{p:03d} T{t:03d}", flush=True)
for _ in range(3):
    send(60); time.sleep(1.4)
    send(120); time.sleep(1.4)
send(90); time.sleep(1.0)
print("PAN test done, both back at center 90/90", flush=True)
ser.close()
