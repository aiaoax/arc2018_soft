#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

import rospy

from tama.msg import sonar

import time
# gpio制御用
import pigpio

PUBLISH_HZ = 0.5
PIN_TRIG = 29
PIN_ECHO = 31
HIGH = 1
LOW  = 0

class Sonar(object):
    def __init__(self):
        rospy.init_node('sonor_node', anonymous=True)
        self.num_readings = 90
        self.duration = 1.0
        self.pin_trig = 0xff
        self.pin_echo = 0xff
        self.pi = pigpio.pi()
        self.range = 0 
        self.msg = sonar()
        self.pub = rospy.Publisher('sonar',sonar, queue_size=100)

    def getDuration(self):
        self.pi.write(self.pin_trig,HIGH)
        time.sleep(1/(1000*100))
        self.pi.write(self.pin_trig,LOW)
        
        while self.pi.read(self.pin_echo) == LOW: pass
        start = self.pi.get_current_tick()
        while self.pi.read(self.pin_echo) == HIGH: pass
        end   = self.pi.get_current_tick()
        duration = end - start
        
    def callback(self,gpio,level,tick):
        self.range = self.getDuration()

    def transmit(self):
        self.msg.range = self.getRange()
        print "range " + str(self.msg.range)
        self.pub.publish(self.msg)
        pass
 
    def setPin(self,trig,echo):
        self.pin_trig = trig
        self.pin_echo = echo
        self.pi.set_mode(trig,pigpio.OUTPUT)
        self.pi.set_mode(echo,pigpio.INPUT)
        #self.callback_tirg = self.pi.callback(trig,pigpio.EITHER_EDGE,self.callback)

    def setRange(self,_range):
        self.range = _range

    def getRange(self):
        self.range = (self.getDuration() * 34300 )/2
        return self.range

def sonar_py():
    # インスタンスの作成
    sonar = Sonar()
    sonar.setPin(PIN_TRIG,PIN_ECHO)
    # 1秒間にpublishする数の設定
    r = rospy.Rate(PUBLISH_HZ)
    print"start sonar"
    # ctl +　Cで終了しない限りwhileループでpublishし続ける
    while not rospy.is_shutdown():
	# publishする関数
	sonar.transmit()
        r.sleep()
        print "roop"

if __name__ == '__main__':
    try:
        sonar_py()

    except rospy.ROSInterruptException: pass
