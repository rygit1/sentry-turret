#!/usr/bin/env python3
"""Jog all three servos. Movement is tied to how long you HOLD the key.
Hold = it moves. Let go = it stops instantly. No runaway.

Click the window first, then:
  LEFT / RIGHT arrows = PAN  (swivel)
  UP / DOWN arrows    = TILT (nod; go gentle)
  A / D               = TRIGGER arm
  [ / ]  = trigger slower / faster    SPACE = recenter pan+tilt (90/72)    Q/Esc = quit

Read your two trigger numbers off the screen:
  REST = arm just clear of the trigger      PULL = trigger squeezed all the way in
Gun OFF + no gel = safe. Needs the firmware with the 'G' command flashed.
"""
import serial, time, sys

ser = serial.Serial("/dev/cu.usbserial-0001", 115200, timeout=1)
time.sleep(2.5)   # let the chip reboot after the port opens

try:
    import pygame
except ImportError:
    ser.close()
    sys.exit("pygame missing: run  .venv/bin/pip install pygame  then try again")

pygame.init()
pygame.display.set_mode((660, 250))
pygame.display.set_caption("Turret jog - arrows=pan/tilt, A/D=trigger")
screen = pygame.display.get_surface()
big = pygame.font.SysFont("menlo", 34)
small = pygame.font.SysFont("menlo", 15)
clock = pygame.time.Clock()

pan, tilt, trig = 90.0, 72.0, 90.0
PT_RATE = 45.0        # pan/tilt degrees per second while held
trig_rate = 25.0      # trigger degrees per second while held (tune with [ ])
last_pt = (-1, -1)
last_trig = -1

def send_pt():
    global last_pt
    p, t = int(round(pan)), int(round(tilt))
    if (p, t) != last_pt:
        ser.write(f"P{p:03d} T{t:03d}\n".encode())
        last_pt = (p, t)

def send_trig():
    global last_trig
    g = int(round(trig))
    if g != last_trig:
        ser.write(f"G{g:03d}\n".encode())
        last_trig = g

send_pt(); send_trig()
running = True
while running:
    dt = clock.tick(60) / 1000.0   # real seconds since the last frame

    for e in pygame.event.get():
        if e.type == pygame.QUIT:
            running = False
        elif e.type == pygame.KEYDOWN:
            if e.key in (pygame.K_q, pygame.K_ESCAPE):
                running = False
            elif e.key == pygame.K_SPACE:
                pan, tilt = 90.0, 72.0
            elif e.key == pygame.K_LEFTBRACKET:
                trig_rate = max(5.0, trig_rate - 5.0)
            elif e.key == pygame.K_RIGHTBRACKET:
                trig_rate = min(60.0, trig_rate + 5.0)

    k = pygame.key.get_pressed()
    if k[pygame.K_RIGHT]: pan  = min(180.0, pan  + PT_RATE * dt)
    if k[pygame.K_LEFT]:  pan  = max(0.0,   pan  - PT_RATE * dt)
    if k[pygame.K_UP]:    tilt = min(180.0, tilt + PT_RATE * dt)
    if k[pygame.K_DOWN]:  tilt = max(0.0,   tilt - PT_RATE * dt)
    if k[pygame.K_d]:     trig = min(180.0, trig + trig_rate * dt)
    if k[pygame.K_a]:     trig = max(0.0,   trig - trig_rate * dt)

    send_pt(); send_trig()

    screen.fill((18, 18, 22))
    screen.blit(big.render(f"PAN {int(round(pan))}   TILT {int(round(tilt))}   TRIG {int(round(trig))}",
                           True, (90, 225, 130)), (24, 52))
    screen.blit(small.render("arrows = pan / tilt      A / D = trigger      hold to move, let go to stop",
                             True, (205, 205, 205)), (24, 140))
    screen.blit(small.render("[ ] trigger slower/faster     SPACE = recenter pan+tilt     Q quit",
                             True, (150, 150, 150)), (24, 166))
    screen.blit(small.render(f"trigger speed {int(trig_rate)} deg/sec   |   REST = arm just clear, PULL = fully squeezed",
                             True, (150, 150, 150)), (24, 192))
    pygame.display.flip()

ser.close()
pygame.quit()
