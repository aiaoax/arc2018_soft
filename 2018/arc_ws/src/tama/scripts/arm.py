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

#Sarvo PWM 
CW_SARVO1 = 1500
CW_SARVO2 = 1500
CCW_SARVO1 = 1000
CCW_SARVO2 = 1000

PLUS_SERVO2 = 2
MINUS_SERVO2 = -2

SOFTPWM_MAX = 255
SOFTPWM_1_3 = (1 / 3) * SOFTPWM_MAX
SOFTPWM_OFF = 0

HIGH = 1
LOW = 0

tilt_pulse_width = CCW_SARVO2

# initialize gpio
pi = pigpio.pi()
pi.set_mode(PIN_SARVO1, pigpio.OUTPUT)
pi.set_mode(PIN_SARVO2, pigpio.OUTPUT)
pi.set_mode(PIN_INCW, pigpio.OUTPUT)
pi.set_mode(PIN_INCCW, pigpio.OUTPUT)

class ArmClass():
    def __init__(self):
        pass
    
    def callback(self,arm):
        duty = ((arm.frame_id % 90.) / 180. * 1.9 % 0.5)\
                / 20. * 1e6
        #pi.hardware_PWM(pwm_pin, 50, 50000)
        print('frame_id = %d ' % arm.frame_id )
        
        #叩く
        self.strikeMotion(arm.strike)
        
        #掴む/離す
        self.grubMotion(arm.grub)
        
        #格納
        self.storeMotion(arm.store)
        
        #ホームに戻す
        self.homeMotion(arm.home)
        
        #アームチルト
        self.tiltMotion(arm.tilt)

        #ベース
        self.baseMotion(arm.updown)

        #解放
        self.releaseMotion(arm.release)

        print("=============")


    def strikeMotion(self,strike):
        if strike:
            #ハンマーを振る
            pwm_duty = SOFTPWM_1_3
            
        else:
            #ハンマーを止める
            pwm_duty = SOFTPWM_OFF
        
        pi.set_PWM_dutycycle(PIN_INCW,pwm_duty) # PWM off
        print("strike = %s\t\tPWM = %d" %  (strike,pwm_duty))
        

    def grubMotion(self,grub):
        
        if grub:
            #掴む
            pwm_width = CW_SARVO1
        else:
            #離す
            pwm_width = CCW_SARVO1
        
        pi.set_servo_pulsewidth(PIN_SARVO1, pwm_width)
        print("grub = %s\t\tPWM = %f" % (grub,pwm_width))

    def storeMotion(self,store):
        if store:
            #
            pi.set_servo_pulsewidth(PIN_SARVO1, CCW_SARVO1)
            pi.set_servo_pulsewidth(PIN_SARVO2, CCW_SARVO2)
        else:
            pass #何もしない
            
        print("store = %s" % store)

    def homeMotion(self,home):
        if home:
            pass #ダミー
        else:
            pass #何もしない
            
        print("home = %s" % home)

    def tiltMotion(self,tilt):
        global tilt_pulse_width
        if (tilt == Arm.PLUS) and (tilt_pulse_width < CW_SARVO2):
            #手首を上向きに
            tilt_pulse_width += PLUS_SERVO2
        elif (tilt == Arm.MINUS) and (tilt_pulse_width > CCW_SARVO2):
            #手首を下向きに
            tilt_pulse_width += MINUS_SERVO2
        else:
            pass
            
        pi.set_servo_pulsewidth(PIN_SARVO2, tilt_pulse_width)
        print("tilt = %s\t\tWidth = %d" % (tilt,tilt_pulse_width))

    def baseMotion(self,updown):
        if (updown == Arm.PLUS):
            #ベース位置を上へ
            pi.write(PIN_INCW,HIGH)
            pi.write(PIN_INCCW,LOW)
        elif (updown == Arm.MINUS):
            #ベース位置を下へ
            pi.write(PIN_INCW,LOW)
            pi.write(PIN_INCCW,HIGH)
        else:
            #ベース位置を固定
            pi.write(PIN_INCW,HIGH)
            pi.write(PIN_INCCW,HIGH)
            
        print("updown = %s" % updown)

    def releaseMotion(self,release):
        if release:
            #手首を一番上向きに
            #離す
            pass #ダミー
        else:
            pass #何もしない
            
        print("release = %s" % release)

def arm_py():
    armc = ArmClass()
    rospy.init_node('arm_py_node',anonymous=True)
    sub=rospy.Subscriber('arm', arm, armc.callback, queue_size=1)
    print "start"
    rospy.spin()

if __name__ == '__main__':
   arm_py()
