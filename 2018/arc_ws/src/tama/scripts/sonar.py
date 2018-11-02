#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

import rospy
import sys
from tama.msg import sonar

import time
# gpio制御用
import pigpio

HZ_PUBLSH = 5
PIN_TRIG = [17,22,9,5]
PIN_ECHO = [27,10,11,6]
HIGH = 1
LOW  = 0

TIME_WATCH_DOG = 100 # 100msec
TIME_TRIG_ON = 1/1000*100 # 10us
SONIC = 340*100/1000*1000 # cm/us
MAX_DURATION  = (450*2)/SONIC
TICK_LIMIT = 4294967295
EDGE_FALLING = 0
EDGE_RISING  = 1

class Sonar(object):
    def __init__(self):
        rospy.init_node('sonor_node', anonymous=True)
        self.num_readings = 90
        self.duration = 1.0
        self.id = 0xff  
        self.pin_trig = 0xff
        self.pin_echo = 0xff
        self.pi = pigpio.pi()
        self.range = 0 
        self.time_start = 0
        self.time_end   = 0
        self.msg = sonar()
        self.pub = rospy.Publisher('sonar',sonar, queue_size=100)

    def callback(self,gpio,level,tick):
        if level == EDGE_FALLING :
            #print "EDGE_FALL " + str(tick) 
            self.time_end = tick
            self.duration = tick - self.time_start
            if self.duration < 0 :
                self.duration += TICK_LIMIT
            self.resetTrig()
        elif level == EDGE_RISING :
            #print "EDGE_RISE " + str(tick) 
            self.time_start = tick
        else :
            self.resetTrig()
            self.duration = MAX_DURATION
            print "TIME OUT"

    def transmit(self):
        self.msg.range = self.getRange()
        print "range " + str(self.msg.range)
        self.pub.publish(self.msg)

    def resetTrig(self) :
        self.pi.write(self.pin_trig,HIGH)
        time.sleep(TIME_TRIG_ON)
        self.pi.write(self.pin_trig,LOW)
        
    def start(self) :
        self.cb = self.pi.callback(self.pin_echo,pigpio.EITHER_EDGE,self.callback)
        print "echo " +str(self.cb)
        self.resetTrig()
        #self.pi.callback(slef.pin_echo,pigpio.RISING_EDGE,callback)

    def stop(self) :
        self.cb.cancel()
        self.pi.stop()
        print "terminate"

    def setId(self,_id):
        self.id = int(_id)
        self.msg.id = int(_id)
        self.setPin(PIN_TRIG[int(_id)],PIN_ECHO[int(_id)])

    def setPin(self,trig,echo):
        self.pin_trig = trig
        self.pin_echo = echo
        self.pi.set_mode(trig,pigpio.OUTPUT)
        self.pi.set_mode(echo,pigpio.INPUT)
        self.pi.write(echo,HIGH)
        self.pi.set_watchdog(echo,TIME_WATCH_DOG )
        #self.callback_tirg = self.pi.callback(trig,pigpio.EITHER_EDGE,self.callback)

    def setRange(self,_range):
        self.range = _range

    def getRange(self):
        self.range = (self.duration * SONIC )/2
        return self.range

def sonar_py():
    # インスタンスの作成
    sonar = Sonar()
    args = sys.argv
    sonar.setId(args[1])
    sonar.start()
    # 1秒間にpublishする数の設定
    r = rospy.Rate(HZ_PUBLSH)
    print"start sonar"
    # ctl +　Cで終了しない限りwhileループでpublishし続ける
    while not rospy.is_shutdown():
        # publishする関数
        sonar.transmit()
        r.sleep()
    sonar.stop()
if __name__ == '__main__':
    try:
        sonar_py()

    except rospy.ROSInterruptException: pass
