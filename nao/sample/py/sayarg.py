#!/usr/bin/python
# -*- coding: utf-8 -*-
from naoqi import ALProxy
from naoconfig import *
import sys,string

proxyAudio = ALProxy("ALTextToSpeech",robot_IP,robot_port)
proxyAudio.say(string.join(sys.argv[1:]))
