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
PIN_AIN1    = 19#16#19 # GPIO.24 Left IN1
PIN_AIN2    = 26#20    # GPIO.27 Left IN2
PIN_PWMA    = 13#12    # GPIO.26 Left PWM
PIN_BIN1    = 20#26    # GPIO.25 Right IN1
PIN_BIN2    = 16#20    # GPIO.28 Right IN2
PIN_PWMB    = 12#13    # GPIO.23 Right PWM

#HIGH_SPD    = 100  # 速度：高, 値の範囲：0~100%
#MIDDLE_SPD  = 75   # 速度：中, 値の範囲：0~100%
#LOW_SPD     = 50   # 速度：低, 値の範囲：0~100%

#HIGH_TURN   = 75   # 旋回速度：高, 値の範囲：0~100%
#MIDDLE_TURN = 60   # 旋回速度：中, 値の範囲：0~100%
#LOW_TURN    = 35   # 旋回速度：低, 値の範囲：0~100%

RIGHT_FIGURE = 1.0  # 右タイヤ回転比係数
LEFT_FIGURE  = 1.0  #0.8  # 右タイヤ回転比係数

SPD_Measure_TERM = 18 # 16.7ms x 18 = 300.6ms
SPD_CONTROL_TERM = 60 # 16.7ms x 60 = 1002ms

HIGH        = 1     # 定数
LOW         = 0     # 定数

LOW_SPD = 0.5       # 50% Speed for Preventing Falling-down

BREAK_CNT_l = 0     # Left Motor Break Counter for ABS  LSB: 16.7ms
BREAK_CNT_r = 0     # Right Motor Break Counter for ABS LSB: 16.7ms

Last_Speed_l = 0    # Left Motor Last Cycle Speed from Joystick
Last_Speed_r = 0    # Right Motor Last Cycle Speed from Joystick

Speed_l = 0         # Left Motor Speed for Control
Speed_r = 0         # Right Motor Speed for Control

Speed_Measure_Term_cnt = 0  # Both Motor Speed Measure Term Counter
Speed_Control_Term_cnt = 0  # Both MOtor Speed Control Term COunter

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
    global BREAK_CNT_l
    global BREAK_CNT_r
    global Last_Speed_l
    global Last_Speed_r
    global Speed_l
    global Speed_r
    global Speed_Measure_Term_cnt
    global Speed_Control_Term_cnt

    print"==============="
    print("frame_id = %d" % foot.frame_id)
    print("direction_l = %d" % foot.direction_l)
    print("direction_r = %d" % foot.direction_r)
    print("speed_l = %d" % foot.speed_l)
    print("speed_r = %d" % foot.speed_r)
    print"==============="

    # Speed Control for Preventing Falling-down
    if (foot.speed_l*Last_Speed_l) < 0 and (foot.speed_r*Last_Speed_r) < 0:  # When the Both Motor Speed Reversal
        Speed_Control_Term_cnt = 1      # Start Counting
    if Speed_Control_Term_cnt > SPD_CONTROL_TERM: # After 1s
        Speed_Control_Term_cnt = 0      # Initialization
    #Speed Control
    if Speed_Control_Term_cnt > 0:      # During 1s
        print"Speed Mode : Half Speed"
        Speed_l = foot.speed_l * LOW_SPD  # 50% Speed for Preventing Falling-down
        Speed_r = foot.speed_r * LOW_SPD  # 50% Speed for Preventing Falling-down
    elif Speed_Control_Term_cnt == 0:   # When Normal
        print"Speed Mode : Normal Speed"
        Speed_l = foot.speed_l          # Normal Speed
        Speed_r = foot.speed_r          # Normal Speed

    #左モータ
    print"==============="
    print"Left Motor"
    print"==============="
    if Speed_l > 0:
        outputDirection(PIN_AIN1, LOW, PIN_AIN2, HIGH)  # Left Motor : CCW
        outputPwm(PIN_PWMA, Speed_l*LEFT_FIGURE)        # 速度
    elif Speed_l < 0:
        outputDirection(PIN_AIN1, HIGH, PIN_AIN2, LOW)  # Left Motor : CW
        outputPwm(PIN_PWMA, abs(Speed_l)*LEFT_FIGURE)   # 速度
    elif Speed_l == 0:                  # added ABS Function
        if BREAK_CNT_l > 24:            # Over 400.8ms
            BREAK_CNT_l = 0             # Initialization
        if BREAK_CNT_l > 21:            # During 350.7ms to 400.8ms
            print"ABS : Lock Mode"
            outputDirection(PIN_AIN1, HIGH, PIN_AIN2, HIGH) # Left Motor : ShortBreak
        elif BREAK_CNT_l <= 21:         # Until 350.7ms
            print"ABS : AntiLock Mode"
            pass                        # Antilock for ABS
        BREAK_CNT_l = BREAK_CNT_l + 1   # Counter for ABS + 16.7ms
    print"==============="

    #右モータ
    print"Right Motor"
    print"==============="
    if Speed_r > 0:
        outputDirection(PIN_BIN1, HIGH, PIN_BIN2, LOW)  # Right Motor : CW
        outputPwm(PIN_PWMB, Speed_r*RIGHT_FIGURE)       # 速度
    elif Speed_r < 0:
        outputDirection(PIN_BIN1, LOW, PIN_BIN2, HIGH)  # Right Motor : CCW
        outputPwm(PIN_PWMB, abs(Speed_r)*RIGHT_FIGURE)  # 速度
    elif Speed_r == 0:                  # added ABS Function
        if BREAK_CNT_r > 24:            # Over 400ms
            BREAK_CNT_r = 0             # Initialization
        if BREAK_CNT_r > 21:            # During 350.7ms to 400.8ms
            print"ABS : Lock Mode"
            outputDirection(PIN_BIN1, HIGH, PIN_BIN2, HIGH) # Right Motor : ShortBreak
        elif BREAK_CNT_r <= 21:         # Until 350.7ms
            print"ABS : AntiLock Mode"
            pass                        # Antilock for ABS
        BREAK_CNT_r = BREAK_CNT_r + 1   # Counter for ABS + 16.7ms
    print"==============="

    if Speed_Measure_Term_cnt == SPD_Measure_TERM:    # Every 300.6ms
        # Last Speed Recording for Preventing Falling-down when sudden speed changes
        Last_Speed_l = foot.speed_l
        Last_Speed_r = foot.speed_r
        Speed_Measure_Term_cnt = 0         # initialization

    Speed_Measure_Term_cnt = Speed_Measure_Term_cnt + 1        # Counting UP as 16.7ms

    if Speed_Control_Term_cnt > 0:         # When not normal
        Speed_Control_Term_cnt = Speed_Control_Term_cnt + 1    # Counting Up


    #前進
#    if foot.direction == Direction.AHEAD:
#        outputDirection(LOW, HIGH, HIGH, LOW)   # Left Motor : CCW, Right Motor : CW

    #後進
#    elif foot.direction == Direction.BACK:
#        outputDirection(HIGH, LOW, LOW, HIGH)   # Left Motor : CW, Right Motor : CCW

#    #右旋回
#    elif foot.direction == Direction.RIGHT:
#        outputDirection(LOW, HIGH, LOW, HIGH)   # Left Motor : CCW, Right Motor : CCW

    #左旋回
#    elif foot.direction == Direction.LEFT:
#        outputDirection(HIGH, LOW, HIGH, LOW)   # Left Motor : CW, Right Motor : CW

    #停止
#    elif foot.direction == Direction.STOP:
#        outputDirection(HIGH, HIGH, HIGH, HIGH) # Left Motor : ShortBreak, Right Motor : ShortBreak


    #速度制御
#    if foot.speed == Speed.HIGH:
#        if foot.direction == Direction.AHEAD or foot.direction == Direction.BACK:
#            outputPwm(HIGH_SPD)         # 速度：高
#        else:
#            outputPwm(HIGH_TURN)        # 旋回速度：高

#    elif foot.speed == Speed.MIDDLE:
#        if foot.direction == Direction.AHEAD or foot.direction == Direction.BACK:
#            outputPwm(MIDDLE_SPD)       # 速度：中
#        else:
#            outputPwm(MIDDLE_TURN)      # 旋回速度：中

#    elif foot.speed == Speed.LOW:
#        if foot.direction == Direction.AHEAD or foot.direction == Direction.BACK:
#            outputPwm(LOW_SPD)          # 速度：低
#        else:
#            outputPwm(LOW_TURN)         # 旋回速度：低

#    else:
#        pass

def outputPwm(PWM, SPD):                     # PWM Duty比
    pi.hardware_PWM(PWM,20*1000, SPD*10*1000)                 # 周波数：20kHz, Duty比：SPD%
    print "SPD   : " + str(pi.get_PWM_dutycycle(PWM))
    print "FRQ   : " + str(pi.get_PWM_frequency(PWM))
    print "RANGE : " + str(pi.get_PWM_range(PWM))
#def outputPwm(SPD):                     # PWM Duty比
#    pi.hardware_PWM(PIN_PWMA,20*1000, SPD*LEFT_FIGURE*10*1000)     # 周波数：20kHz, Duty比：100%
#    pi.hardware_PWM(PIN_PWMB,20*1000, SPD*RIGHT_FIGURE*10*1000)    # 周波数：20kHz, Duty比：100%
#    print "SPD_A " + str(pi.get_PWM_dutycycle(PIN_PWMA))
#    print "SPD_B " + str(pi.get_PWM_dutycycle(PIN_PWMB))
#    print "FRQ_ " + str(pi.get_PWM_frequency(PIN_PWMB))
#    print "RANGE_ " + str(pi.get_PWM_range(PIN_PWMB))

def outputDirection(IN1_P, IN1_D, IN2_P, IN2_D):     # 方向
    #モータ回転方向
    pi.write(IN1_P,IN1_D)
    pi.write(IN2_P,IN2_D)
    print "IN1 " + str (pi.read(IN1_P))
    print "IN2 " + str (pi.read(IN2_P))
#def outputDirection(AIN1, AIN2, BIN1, BIN2):        # 方向
    #Left Motor
    #pi.write(PIN_AIN1,AIN1)
    #pi.write(PIN_AIN2,AIN2)
    #print "AIN1 " + str (pi.read(PIN_AIN1))
    #print "AIN2 " + str (pi.read(PIN_AIN2))
    #Right Motor
    #pi.write(PIN_BIN1,BIN1)
    #pi.write(PIN_BIN2,BIN2)

    #print "BIN1 " + str (pi.read(PIN_BIN1))
    #print "BIN2 " + str (pi.read(PIN_BIN2))
def foot_py():
    rospy.init_node('foot_py_node',anonymous=True)
    sub=rospy.Subscriber('foot', foot, callback, queue_size=1)
    print "start"
    rospy.spin()

if __name__ == '__main__':
    foot_py()
