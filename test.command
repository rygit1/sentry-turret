#!/bin/bash
# Double-click this in Finder to test the turret on your webcam. No hardware needed.
cd "$(dirname "$0")" || exit 1
echo "────────────────────────────────────────────────────"
echo "  SENTRY TURRET - webcam test (no parts needed)"
echo "────────────────────────────────────────────────────"
echo "  A window opens using your webcam."
echo
echo "  GREEN box + reticle  = locked onto YOU"
echo "  RED box              = not you (ignored)"
echo
echo "  TO SEE IT 'FIRE':"
echo "   1) press SPACE to ARM (top bar turns red: ARMED)"
echo "   2) face the camera and hold still until the reticle"
echo "      turns YELLOW and reads LOCKED"
echo "   3) a big red FIRE flashes = it shot. (simulated -"
echo "      no real gun yet, this is the brain only)"
echo
echo "  Keys:  SPACE = arm/disarm    Q = quit"
echo "  (first run, macOS asks for Camera permission - click Allow, then reopen)"
echo
exec .venv/bin/python turret_brain.py --lock-me "$@"
