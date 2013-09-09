#!/usr/bin/python
# -*- coding: utf-8 -*-
from head import *

#servo_on()

#forward(0.1)
#turn_left(90.0)
#forward(0.1)
#turn_left(90.0)

#print get_sonar_l()
#print get_sonar_r()

servo_on()
for i in range(0,10):
  print get_sonar_l()
  #forward(0.1)
  if float(get_sonar_l())>0.6:
    print "owari"
    break
