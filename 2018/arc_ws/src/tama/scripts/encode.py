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
CW_SARVO2 = 1500
CCW_SARVO1 = 1000
CCW_SARVO2 = 1000

SOFTPWM_MAX = 255
SOFTPWM_1_3 = 3 / SOFTPWM_MAX
SOFTPWM_OFF = 0

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
    
    #叩く
    strikeMotion(arm.strike)
    
    #掴む/離す
    grubMotion(arm.grub)
    
    #格納
    storeMotion(arm.store)
    
    #ホームに戻す
    homeMotion(arm.home)
    
    #アームチルト
    tiltMotion(arm.tilt)

    #ベース
    baseMotion(arm.updown)

    #解放
    releaseMotion(arm.release)

    print("=============")


def strikeMotion(strike):
    if strike:
        pwm_duty = SOFTPWM_1_3
        
    else:
        pwm_duty = SOFTPWM_OFF
    
    pi.set_PWM_dutycycle(PIN_INCW,pwm_duty) # PWM off
    print("strike = %s PWM = %d" %  (strike,pwm_duty))
    

def grubMotion(grub):
    if grub:
        pwm_width = CW_SARVO1
    else:
        pwm_width = CCW_SARVO1
    
    pi.set_servo_pulsewidth(PIN_SARVO1, pwm_width)
    print("grub = %s PWM = %f" % (grub,pwm_width))

def storeMotion(store):
    if store:
        pi.set_servo_pulsewidth(PIN_SARVO1, CCW_SARVO1)
        pi.set_servo_pulsewidth(PIN_SARVO2, CCW_SARVO2)
    else:
        pass #何もしない
        
    print("store = %s" % store)

def homeMotion(home):
    if home:
        pass #ダミー
    else:
        pass #何もしない
        
    print("home = %s" % home)

def tiltMotion(tilt):
    if tilt == Arm.PLUS:
        pi.set_servo_pulsewidth(PIN_SARVO2, CW_SARVO2)
    elif tilt == Arm.MINUS:
        pi.set_servo_pulsewidth(PIN_SARVO2, CCW_SARVO2)
    else:
        pass
        
    print("tilt = %s" % tilt)

def baseMotion(updown):
    if updown == Arm.PLUS:
        pi.write(PIN_INCW,HIGH)
        pi.write(PIN_INCCW,LOW)
    elif updown == Arm.MINUS:
        pi.write(PIN_INCW,LOW)
        pi.write(PIN_INCCW,HIGH)
    else:
        pi.write(PIN_INCW,HIGH)
        pi.write(PIN_INCCW,HIGH)
        
    print("updown = %s" % updown)

def releaseMotion(release):
    if release:
        pass #ダミー
    else:
        pass #何もしない
        
    print("release = %s" % release)

def arm_py():
    rospy.init_node('arm_py_node',anonymous=True)
    sub=rospy.Subscriber('arm', arm, callback, queue_size=1)
    print "start"
    rospy.spin()

if __name__ == '__main__':
   arm_py()
