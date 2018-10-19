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
is_strike = False

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
        #ボタンを押すたび切り替える
        global is_strike
        if strike:
            #切り替える
            is_strike = not(is_strike)
        else:
            pass #切り替えない
        
        if is_strike:
            #ハンマーを振る
            pwm_duty = SOFTPWM_1_3
            
        else:
            #ハンマーを止める
            pwm_duty = SOFTPWM_OFF
        
        pi.set_PWM_dutycycle(PIN_INCW,pwm_duty)
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
        #[格納]ってそもそも必要なの？
        if store:
            pi.set_servo_pulsewidth(PIN_SARVO1, CCW_SARVO1)
            pi.set_servo_pulsewidth(PIN_SARVO2, CCW_SARVO2)
        else:
            pass #何もしない
            
        print("store = %s" % store)

    def homeMotion(self,home):
        if home:
            # ベースを一番上に
            # ...するらしい
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
            pass #止める
            
        pi.set_servo_pulsewidth(PIN_SARVO2, tilt_pulse_width)
        print("tilt = %s\t\tWidth = %d" % (tilt,tilt_pulse_width))

    def baseMotion(self,updown):
        pwm_duty_cw = SOFTPWM_MAX
        pwm_duty_ccw = SOFTPWM_MAX
        
        if (updown == Arm.PLUS):
            #ベース位置を上へ
            pwm_duty_ccw = SOFTPWM_OFF
        elif (updown == Arm.MINUS):
            #ベース位置を下へ
            pwm_duty_cw = SOFTPWM_OFF
        else:
            #ベース位置を固定
            pass #何もしない

        pi.set_PWM_dutycycle(PIN_INCW,pwm_duty_cw)
        pi.set_PWM_dutycycle(PIN_INCCW,pwm_duty_ccw)
        
        print("updown = %s cw:%d ccw:%d" % (updown,pwm_duty_cw,pwm_duty_ccw))

    def releaseMotion(self,release):
        if release:
            #ベースを一番下に
            #...するのかな？

            #手首を一番上向きに
            pi.set_servo_pulsewidth(PIN_SARVO2, PLUS_SERVO2)
            
            #離す
            pi.set_servo_pulsewidth(PIN_SARVO1, CCW_SARVO1)

        else:
            pass #何もしない
        
        print("release = %s" % release)

def arm_py():
    armc = ArmClass()
    rospy.init_node('arm_py_node',anonymous=True)
    sub=rospy.Subscriber('arm', arm, armc.callback, queue_size=1)
    print ("start")
    rospy.spin()

if __name__ == '__main__':
   arm_py()
