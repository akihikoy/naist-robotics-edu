#!/usr/bin/python
from naoqi import ALProxy
from naoconfig import *

proxySonar = ALProxy("ALSonar",robot_IP,robot_port)
proxySonar.subscribe("my_sonar")

proxyMem = ALProxy("ALMemory",robot_IP,robot_port)

while True:
  l= proxyMem.getData("Device/SubDeviceList/US/Left/Sensor/Value")
  r= proxyMem.getData("Device/SubDeviceList/US/Right/Sensor/Value")
  print str(l)+" : "+str(r)
