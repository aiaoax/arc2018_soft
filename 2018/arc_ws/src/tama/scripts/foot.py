#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Reference:https://karaage.hatenadiary.jp/entry/2017/02/10/073000
Summery: Move servo angle to the topic value 'servo_angle'
"""

import pigpio
import rospy
from tama.msg import foot
from param import Direction
from param import Speed
# defined const

# pin number
PIN_AIN1    = 20#19 # GPIO.24 Left IN1
PIN_AIN2    = 16    # GPIO.27 Left IN2 
PIN_PWMA    = 12    # GPIO.26 Left PWM
PIN_BIN1    = 26    # GPIO.25 Right IN1
PIN_BIN2    = 19#20 # GPIO.28 Right IN2
PIN_PWMB    = 13    # GPIO.23 Right PWM

HIGH_SPD    = 100   # 速度：高, 値の範囲：0~100%
MIDDLE_SPD  = 75   # 速度：中, 値の範囲：0~100%
LOW_SPD     = 50   # 速度：低, 値の範囲：0~100%

HIGH_TURN   = 75   # 旋回速度：高, 値の範囲：0~100%
MIDDLE_TURN = 60   # 旋回速度：中, 値の範囲：0~100%
LOW_TURN    = 35   # 旋回速度：低, 値の範囲：0~100%

RIGHT_FIGURE = 1.0  # 右タイヤ回転比係数

HIGH        = 1     # 定数
LOW         = 0     # 定数

# initialize gpio
pi = pigpio.pi()
pi.set_mode(PIN_AIN1, pigpio.OUTPUT)
pi.set_mode(PIN_AIN2, pigpio.OUTPUT)
pi.set_mode(PIN_PWMA, pigpio.OUTPUT)
pi.set_mode(PIN_BIN1, pigpio.OUTPUT)
pi.set_mode(PIN_BIN2, pigpio.OUTPUT)
pi.set_mode(PIN_PWMB, pigpio.OUTPUT)
#pi.set_PWM_frequency(PIN_PWMA,1000)
#pi.set_PWM_frequency(PIN_PWMB,1000)
def callback(foot):
    print("frame_id = %d" % foot.frame_id)
    print("direction = %d" % foot.direction)
    print("speed = %d" % foot.speed)
    print"==============="

    #方向制御
    #前進
    if foot.direction == Direction.AHEAD:
        outputDirection(LOW, HIGH, HIGH, LOW)   # Left Motor : CCW, Right Motor : CW

    #後進
    elif foot.direction == Direction.BACK:
        outputDirection(HIGH, LOW, LOW, HIGH)   # Left Motor : CW, Right Motor : CCW

    #右旋回
    elif foot.direction == Direction.RIGHT:
        outputDirection(LOW, HIGH, LOW, HIGH)   # Left Motor : CCW, Right Motor : CCW

    #左旋回
    elif foot.direction == Direction.LEFT:
        outputDirection(HIGH, LOW, HIGH, LOW)   # Left Motor : CW, Right Motor : CW

    #停止
    elif foot.direction == Direction.STOP:
        outputDirection(HIGH, HIGH, HIGH, HIGH) # Left Motor : ShortBreak, Right Motor : ShortBreak


    #速度制御
    if foot.speed == Speed.HIGH:
        if foot.direction == Direction.AHEAD or foot.direction == Direction.BACK:
            outputPwm(HIGH_SPD)         # 速度：高
        else:
            outputPwm(HIGH_TURN)        # 旋回速度：高

    elif foot.speed == Speed.MIDDLE:
        if foot.direction == Direction.AHEAD or foot.direction == Direction.BACK:
            outputPwm(MIDDLE_SPD)       # 速度：中
        else:
            outputPwm(MIDDLE_TURN)      # 旋回速度：中

    elif foot.speed == Speed.LOW:
        if foot.direction == Direction.AHEAD or foot.direction == Direction.BACK:
            outputPwm(LOW_SPD)          # 速度：低
        else:
            outputPwm(LOW_TURN)         # 旋回速度：低

    else:
        pass

def outputPwm(SPD):                     # PWM Duty比
#    pi.set_PWM_dutycycle(PIN_PWMA, SPD)
#    pi.set_PWM_dutycycle(PIN_PWMB, SPD * RIGHT_FIGURE)
    pi.hardware_PWM(PIN_PWMA,20*1000, SPC*10*1000)                 # 周波数：20kHz, Duty比：100%
    pi.hardware_PWM(PIN_PWMB,20*1000, SPC*RIGHT_FIGURE*10*1000)    # 周波数：20kHz, Duty比：100%
    print "SPD_A " + str(pi.get_PWM_dutycycle(PIN_PWMA))
    print "SPD_B " + str(pi.get_PWM_dutycycle(PIN_PWMB))
    print "FRQ_ " + str(pi.get_PWM_frequency(PIN_PWMB))
    print "RANGE_ " + str(pi.get_PWM_range(PIN_PWMB))
    

def outputDirection(AIN1, AIN2, BIN1, BIN2):    # 方向
    #Left Motor
    pi.write(PIN_AIN1,AIN1)
    pi.write(PIN_AIN2,AIN2)
    print "AIN1 " + str (pi.read(PIN_AIN1))
    print "AIN2 " + str (pi.read(PIN_AIN2))
    #Right Motor
    pi.write(PIN_BIN1,BIN1)
    pi.write(PIN_BIN2,BIN2)

    print "BIN1 " + str (pi.read(PIN_BIN1))
    print "BIN2 " + str (pi.read(PIN_BIN2))
def foot_py():
    rospy.init_node('foot_py_node',anonymous=True)
    sub=rospy.Subscriber('foot', foot, callback, queue_size=1)
    print "start"
    rospy.spin()

if __name__ == '__main__':
    foot_py()
