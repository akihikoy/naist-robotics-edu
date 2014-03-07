#!/usr/bin/python
# -*- coding: utf-8 -*-
from naoqi import ALProxy
from naoconfig import *

#!/usr/bin/python
# -*- coding: utf-8 -*-

class _Getch:
    """Gets a single character from standard input.  Does not echo to the screen."""
    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self): return self.impl()


class _GetchUnix:
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class _GetchWindows:
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()

getch = _Getch()


proxyMo = ALProxy('ALMotion',robot_IP,robot_port)
proxyMo.stiffnessInterpolation('Body', 1.0, 1.0)


# Example showing how to set LArm Position, using a fraction of max speed
chainName = "LArm"
space     = 0 # FRAME_TORSO
useSensor = False

# Get the current position of the chainName in the same space
target = proxyMo.getPosition(chainName, space, useSensor)

while True:
  delta= 0.002
  key= getch()
  dx=0
  dy=0
  dz=0
  if ord(key[0])==65: dy+=delta
  elif ord(key[0])==66: dy-=delta
  elif ord(key[0])==67: dx-=delta
  elif ord(key[0])==68: dx+=delta
  elif ord(key[0])==72: dz+=delta
  elif ord(key[0])==70: dz-=delta
  elif key=='q': break

  target  = [
      target[0] + dx,
      target[1] + dy,
      target[2] + dz,
      target[3] + 0.0,
      target[4] + 0.0,
  target[5] + 0.0]
  print target

  fractionMaxSpeed = 0.5
  axisMask         = 7 # just control position

  proxyMo.setPosition(chainName, space, target, fractionMaxSpeed, axisMask)

