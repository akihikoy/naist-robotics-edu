#!/usr/bin/python
# -*- coding: utf-8 -*-
from naoqi import ALProxy
from naoconfig import *
import math

proxyMo = ALProxy('ALMotion',robot_IP,robot_port)

proxySonar = ALProxy("ALSonar",robot_IP,robot_port)
proxySonar.subscribe("my_sonar")
proxyMem = ALProxy("ALMemory",robot_IP,robot_port)

def servo_off():
  global proxyMo
  proxyMo.stiffnessInterpolation('Body', 0.0, 1.0)

def servo_on():
  global proxyMo
  proxyMo.stiffnessInterpolation('Body', 1.0, 1.0)

def walk_to(x,y,theta):
  global proxyMo
  proxyMo.walkTo(x, y, theta)

def forward(x):
  walk_to(x,0.0,0.0)

def turn_left(angle):
  walk_to(0.0,0.0,angle*math.pi/180.0)

def turn_right(angle):
  walk_to(0.0,0.0,-angle*math.pi/180.0)

def get_sonar_l():
  return proxyMem.getData("Device/SubDeviceList/US/Left/Sensor/Value")

def get_sonar_r():
  return proxyMem.getData("Device/SubDeviceList/US/Right/Sensor/Value")
