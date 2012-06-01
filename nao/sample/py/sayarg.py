#!/usr/bin/python
# -*- coding: utf-8 -*-
from naoqi import ALProxy
from naoconfig import *
import sys,string

proxyAudio = ALProxy("ALTextToSpeech",robot_IP,robot_port)
if len(sys.argv)>1:
  proxyAudio.say(string.join(sys.argv[1:]))
else:
 while 1:
  sys.stdout.write('>> ')
  text= sys.stdin.readline().strip()
  if text=='q':  break
  proxyAudio.post.say(text)
