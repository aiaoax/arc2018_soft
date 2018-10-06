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
PIN_AIN1    = 25    # Left IN1
PIN_AIN2    = 24    # Left IN2
PIN_PWMA    = 23    # Left PWM
PIN_BIN1    = 28    # Right IN1
PIN_BIN2    = 27    # Right IN2
PIN_PWMB    = 26    # Right PWM

HIGHSPD     = 90    # 速度：高
MEDIUMSPD   = 60    # 速度：中
LOWSPD      = 30    # 速度：低

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

def callback(foot):
    #Comment out
    #duty = ((foot.frame_id + 90.) / 180. * 1.9 + 0.5)\
    #        / 20. * 1e6
    #pi.hardware_PWM(pwm_pin, 50, int(0))
    print("frame_id = %d" % foot.frame_id)
    print("direction = %d" % foot.direction)
    print("speed = %d" % foot.speed)
    print"==============="

    #方向制御
    #前進
    if foot.direction == Direction.AHEAD:
        #Left Motor : CCW
        pi.write(PIN_AIN1,LOW)
        pi.write(PIN_AIN2,HIGH)
        #Right Motor : CW
        pi.write(PIN_BIN1,HIGH)
        pi.write(PIN_BIN2,LOW)

    #後進
    elif foot.direction == Direction.BACK:
        #Left Motor : CW
        pi.write(PIN_AIN1,HIGH)
        pi.write(PIN_AIN2,LOW)
        #Right Motor : CCW
        pi.write(PIN_BIN1,LOW)
        pi.write(PIN_BIN2,HIGH)

    #右旋回
    elif foot.direction == Direction.RIGHT:
        #Left Motor : CCW
        pi.write(PIN_AIN1,LOW)
        pi.write(PIN_AIN2,HIGH)
        #Right Motor : CCW
        pi.write(PIN_BIN1,LOW)
        pi.write(PIN_BIN2,HIGH)

    #左旋回
    elif foot.direction == Direction.LEFT:
        #Left Motor : CW
        pi.write(PIN_AIN1,HIGH)
        pi.write(PIN_AIN2,LOW)
        #Right Motor : CW
        pi.write(PIN_BIN1,HIGH)
        pi.write(PIN_BIN2,LOW)

    #停止
    elif foot.direction == Direction.STOP:
        pass
    else:
        pass


    #速度制御
    if foot.speed == Speed.HIGH:
        pi.write(PIN_PWMA,HIGHSPD)
        pi.write(PIN_PWMB,HIGHSPD)

    elif foot.speed == Speed.MIDDLE:
        pi.write(PIN_PWMA,MEDIUMSPD)
        pi.write(PIN_PWMB,MEDIUMSPD)

    elif foot.speed == Speed.LOW:
        pi.write(PIN_PWMA,LOWSPD)
        pi.write(PIN_PWMB,LOWSPD)

    else:
        pass

def foot_py():
    rospy.init_node('foot_py_node',anonymous=True)
    sub=rospy.Subscriber('foot', foot, callback, queue_size=1)
    print "start"
    rospy.spin()

if __name__ == '__main__':
   foot_py()