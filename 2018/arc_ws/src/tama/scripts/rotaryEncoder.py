#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

# gpio制御用
import pigpio

class RotaryEncoder(object):
    def __init__(self):
        self.current = 0
        self.previous = 0
        self.direction_judge = [ 0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0 ]
        self.pin_a = 0
        self.pin_b = 0
        self.pi = pigpio.pi()
        self.rotate = 0

    def getSum(self):
        #print "a " +str(self.pi.read(self.pin_a))+ "b "+str (self.pi.read(self.pin_b))
        sum_param = (self.pi.read(self.pin_a)<<1) + self.pi.read(self.pin_b)
        return sum_param

    def getDirection(self):
        self.previous = self.current
        self.current = self.getSum()
        #print "pre "+str(self.previous) + "cur " + str(self.current)
        index = (self.previous << 2) + self.current
        #print "index " + str(index)
        direction = self.direction_judge[index]
        return direction

    def callback(self,gpio,level,tick):
        self.rotate += self.getDirection()

    def setPin(self,a,b):
        self.pin_a = a
        self.pin_b = b
        self.pi.set_mode(a,pigpio.INPUT)
        self.pi.set_mode(b,pigpio.INPUT)
        self.current = self.getSum()
        self.callback_a = self.pi.callback(a,pigpio.EITHER_EDGE,self.callback)
        self.callback_b = self.pi.callback(b,pigpio.EITHER_EDGE,self.callback)

    def setRotate(self,rotate):
        self.rotate = rotate

    def getRotate(self):
        return self.rotate
