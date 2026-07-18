#!/usr/bin/env python3
"""Gentle trigger-servo test WITH a live readout from the chip.
Pan/tilt stay parked (no sweeping). Press ENTER = one trigger pull. Ctrl-C = quit.

The chip now prints WHY it did or didn't fire after each ENTER, e.g.:
    [chip] FIRE rx | armed=1 link=1 cooldownOk=1
  armed=0  -> pin 33 is NOT grounded (the "ready-to-fire" jumper isn't reaching ground)
  armed=1 and "-> FIRING" but no movement -> the arm is jammed, give it room to swing

NEEDS the debug firmware flashed. No gel loaded + arm free = safe dry run.
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

time.sleep(2.5)              # let the chip reboot after the port opens
ser.write(b"P090 T072\n")   # park pan/tilt at center; they will not move

print("Ready. Press ENTER to fire one pull. Ctrl-C to quit.")
try:
    while True:
        input()
        ser.write(b"P090 T072 FIRE\n")
except KeyboardInterrupt:
    pass
finally:
    ser.close()
    print("\ndone")
