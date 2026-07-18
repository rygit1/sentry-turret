#!/usr/bin/env python3
"""Lock both servos to dead center (90/90) and confirm they're powered + stiff.
Servos hold their last commanded angle as long as the ESP32 stays powered (USB),
so after this exits they stay centered and stiff. Re-run anytime to re-center."""
import serial, time

PORT = "/dev/cu.usbserial-0001"
ser = serial.Serial(PORT, 115200, timeout=1)
time.sleep(2.5)  # ESP32 reboots when the port opens; wait for it

def send(p, t):
    ser.write(f"P{p:03d} T{t:03d}\n".encode())
    print(f"sent P{p:03d} T{t:03d}", flush=True)

# nudge off-center then snap to 90 so you SEE them move into center (proof they're live)
send(75, 75); time.sleep(0.8)
send(105, 105); time.sleep(0.8)
send(90, 90);  time.sleep(1.0)
print("BOTH SERVOS NOW AT CENTER (90/90) AND HOLDING.", flush=True)
print("They are powered = stiff: try to turn a shaft by hand, it should resist.", flush=True)
ser.close()
print("done (servos stay centered + stiff as long as the USB cable is plugged in)")
