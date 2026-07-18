#!/usr/bin/env python3
"""Trigger-servo ANGLE FINDER. Moves ONLY the trigger servo (pan and tilt are untouched).
Type an angle 0-180 and press ENTER to move the trigger arm there. Type q to quit.

Find your two numbers:
  REST = arm just clear of the trigger, not touching it
  PULL = trigger squeezed all the way in (the fire point)
Then tell Claude both numbers and he'll bake them into the firmware.

NEEDS the firmware with the 'G' command. Gun OFF + no gel loaded = safe.
"""
import serial, time, threading

ser = serial.Serial("/dev/cu.usbserial-0001", 115200, timeout=1)

def reader():
    while True:
        try:
            msg = ser.readline().decode(errors="replace").strip()
        except Exception:
            break
        if msg:
            print("   [chip] " + msg)

threading.Thread(target=reader, daemon=True).start()
time.sleep(2.5)   # let the chip reboot after the port opens

print("Type an angle 0-180 and press ENTER to move the trigger. 'q' to quit.")
print("Small steps. Find REST (arm just clear) and PULL (trigger fully squeezed).")
try:
    while True:
        s = input("angle> ").strip().lower()
        if s in ("q", "quit", "exit"):
            break
        if not s.isdigit():
            continue
        ang = max(0, min(180, int(s)))
        ser.write(f"G{ang:03d}\n".encode())
        print(f"  -> moving trigger to {ang}")
except (KeyboardInterrupt, EOFError):
    pass
finally:
    ser.close()
    print("\ndone")
