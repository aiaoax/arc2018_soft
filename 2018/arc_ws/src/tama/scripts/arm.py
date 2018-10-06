#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Reference:https://karaage.hatenadiary.jp/entry/2017/02/10/073000
Summery: Move servo angle to the topic value 'servo_angle'
"""

import pigpio
import rospy
from tama.msg import arm
from param import Arm

# defined const

# pin number
PIN_SARVO1 = 4
PIN_SARVO2 = 14
PIN_INCW = 15
PIN_INCCW = 18


CW_SARVO1 = 1500
CW_SARVO2 = 180
CCW_SARVO1 = 1000
CCW_SARVO2 = 0

HIGH = 1
LOW = 0

# initialize gpio
pi = pigpio.pi()
pi.set_mode(PIN_SARVO1, pigpio.OUTPUT)
pi.set_mode(PIN_SARVO2, pigpio.OUTPUT)
pi.set_mode(PIN_INCW, pigpio.OUTPUT)
pi.set_mode(PIN_INCCW, pigpio.OUTPUT)

def callback(arm):
    duty = ((arm.frame_id % 90.) / 180. * 1.9 % 0.5)\
            / 20. * 1e6
    #pi.hardware_PWM(pwm_pin, 50, 50000)
    print('frame_id = %d ' % arm.frame_id )
    print("strike = %s" %  arm.strike )
    print("grub = %s" % arm.grub )
    print("store = %s" % arm.store )
    print("home = %s" % arm.home )
    print("tilt = %s" % arm.tilt )
    print("updown = %s" % arm.updown )
    print("release = %s" % arm.release)
    print("=============")
    
    #叩く
    if arm.strike:
        pi.write(PIN_INCW,HIGH)
    else:
        pi.write(PIN_INCW,LOW)
    
    #掴む/離す
    if arm.grub:
        pi.set_servo_pulsewidth(PIN_SARVO1, CW_SARVO1)
    else:
        pi.set_servo_pulsewidth(PIN_SARVO1, CCW_SARVO1)
    
    #格納
    if arm.store:
        pi.set_servo_pulsewidth(PIN_SARVO1, CCW_SARVO1)
        pi.set_servo_pulsewidth(PIN_SARVO2, CCW_SARVO2)
    else:
        pass #何もしない
    
    #ホームに戻す
    if arm.home:
        pass #ダミー
    else:
        pass #何もしない

    #アームチルト
    if arm.tilt == Arm.PLUS:
        pi.set_servo_pulsewidth(PIN_SARVO2, CW_SARVO2)
    elif arm.tilt == Arm.MINUS:
        pi.set_servo_pulsewidth(PIN_SARVO2, CCW_SARVO2)
    else:
        pass

    #ベース
    if arm.updown == Arm.PLUS:
        pi.write(PIN_INCW,HIGH)
        pi.write(PIN_INCCW,LOW)
    elif arm.tilt == Arm.MINUS:
        pi.write(PIN_INCW,LOW)
        pi.write(PIN_INCCW,HIGH)
    else:
        pi.write(PIN_INCW,HIGH)
        pi.write(PIN_INCCW,HIGH)
    
    #解放
    if arm.release:
        pass #ダミー
    else:
        pass #何もしない




def arm_py():
    rospy.init_node('arm_py_node',anonymous=True)
    sub=rospy.Subscriber('arm', arm, callback, queue_size=1)
    print "start"
    rospy.spin()

if __name__ == '__main__':
   arm_py()
