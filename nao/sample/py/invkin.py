#!/usr/bin/python
# -*- coding: utf-8 -*-
from naoqi import ALProxy
from naoconfig import *

#From NAO Document

import sys
import time
from naoqi import ALProxy
from naoqi import motion

try:
    motionProxy = ALProxy("ALMotion", robot_IP,robot_port)
except Exception,e:
    print "Could not create proxy to ALMotion"
    print "Error was: ",e
    sys.exit(1)

motionProxy.stiffnessInterpolation('Body', 1.0, 1.0)

# Example showing how to set LArm Position, using a fraction of max speed
chainName = "LArm"
space     = 0 # FRAME_TORSO
useSensor = False

# Get the current position of the chainName in the same space
current = motionProxy.getPosition(chainName, space, useSensor)

target  = [
    current[0] + 0.1,
    current[1] + 0.1,
    current[2] + 0.1,
    current[3] + 0.0,
    current[4] + 0.0,
current[5] + 0.0]

fractionMaxSpeed = 0.5
axisMask         = 7 # just control position

motionProxy.setPosition(chainName, space, target, fractionMaxSpeed, axisMask)

time.sleep(1.0)

# Example showing how to set Torso Position, using a fraction of max speed
chainName        = "Torso"
space            = 2
position         = [0.0, 0.0, 0.25, 0.0, 0.0, 0.0] # Absolute Position
fractionMaxSpeed = 0.2
axisMask         = 63
motionProxy.setPosition(chainName, space, position, fractionMaxSpeed, axisMask)

