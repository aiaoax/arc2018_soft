#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=C0103
"""
アーム
"""

import pigpio
import rospy
from tama.msg import arm
from param import Arm
from param import Mode


# defined const

DEBUG = 0

UNKNOWN = 0

# pin number
PIN_SARVO1 = 4
PIN_SARVO2 = 14
PIN_INCW = 15
PIN_INCCW = 18

#Sarvo PWM
CW_SARVO1 = 2400
CW_SARVO2 = 2400
CCW_SARVO1 = 550
CCW_SARVO2 = 550

PLUS_SERVO2 = 50

SOFTPWM_W_MAX = 255
SOFTPWM_W_2_5 = (2/5.0) * SOFTPWM_W_MAX
SOFTPWM_W_1_3 = (1/3.0) * SOFTPWM_W_MAX
SOFTPWM_W_OFF = 0

SOFTPWM_F_20K = (20 * 1000)

HIGH = 1
LOW = 0

class ArmClass():
    """
    アームを動かすためのクラス
    """

    def __init__(self):

        # initialize gpio
        self.pic = pigpio.pi()
        self.pic.set_mode(PIN_SARVO1, pigpio.OUTPUT)
        self.pic.set_mode(PIN_SARVO2, pigpio.OUTPUT)
        self.pic.set_mode(PIN_INCW, pigpio.OUTPUT)
        self.pic.set_mode(PIN_INCCW, pigpio.OUTPUT)

        self.pic.set_PWM_dutycycle(PIN_INCCW, SOFTPWM_W_OFF)
        self.pic.set_PWM_frequency(PIN_INCW, SOFTPWM_F_20K)
        self.pic.set_PWM_frequency(PIN_INCCW, SOFTPWM_F_20K)

        self.tilt_pulse_width = ((CW_SARVO2 + CCW_SARVO2) / 2)
        self.mode_old = UNKNOWN

        self.modeChange(UNKNOWN)


    def callback(self, armmes):
        """
        メッセージを受信したときに呼び出し
        """

        print('frame_id = %d ' % armmes.frame_id)

        #モード変更確認
        self.modeChange(armmes.mode)

        #叩く
        self.strikeMotion(armmes.strike)

        #掴む/離す
        self.grubMotion(armmes.grub)

        #格納
        self.storeMotion(armmes.store)

        #ホームに戻す
        self.homeMotion(armmes.home)

        #アームチルト
        self.tiltMotion(armmes.tilt)

        #ベース
        self.baseMotion(armmes.updown)

        #解放
        self.releaseMotion(armmes.release)

        #区切り
        print("==============================")


    def modeChange(self, mode):
        """
        モード変更処理
        """

        if mode != self.mode_old:

            #現在のモードを書き込む
            try:
                file = open('/usr/local/www/mode.txt', 'w')
                tmp_str = "UNKNOWN"

                #モードが増えたら足していこう...
                #ほかにいい方法ないのかを検討
                if mode == Mode.HERVEST:
                    tmp_str = "HERVEST"
                elif mode == Mode.BULB:
                    tmp_str = "BULB"

                file.write(tmp_str)

            finally:
                file.close()

            #モード変更時初期化

            #モーター停止
            self.pic.set_PWM_dutycycle(PIN_INCW, SOFTPWM_W_OFF)
            self.pic.set_PWM_dutycycle(PIN_INCCW, SOFTPWM_W_OFF)

            #サーボの位置調整
            #self.pic.set_servo_pulsewidth(PIN_SARVO1, ((CW_SARVO1 + CCW_SARVO1) / 2))
            #self.pic.set_servo_pulsewidth(PIN_SARVO2, ((CW_SARVO2 + CCW_SARVO2) / 2))

            self.mode_old = mode

        else:
            pass

        print("mode = %s" % self.mode_old)


    def strikeMotion(self, strike):
        """
        ハンマーを振る/止める
        """

        if self.mode_old == Mode.BULB:
            #バルブモード時、〇ボタンで振り始め、×ボタンで止める
            #切り替え判定はbrainで実施
            if strike:
                #ハンマーを振る
                pwm_duty = SOFTPWM_W_2_5
            else:
                #ハンマーを止める
                pwm_duty = SOFTPWM_W_OFF

            self.pic.set_PWM_dutycycle(PIN_INCW, pwm_duty)
            print("strike = %s\t\tPWM = %f" %  (strike, pwm_duty))

        else:
            pass #バルブモード以外は無視


    def grubMotion(self, grub):
        """
        掴む/放す
        """

        if self.mode_old == Mode.HERVEST:
            if grub:
                #掴む
                pwm_width = CW_SARVO1
            else:
                #離す
                pwm_width = CCW_SARVO1

            self.pic.set_servo_pulsewidth(PIN_SARVO1, pwm_width)
            print("grub = %s\t\tPWM = %f" % (grub, pwm_width))

        else:
            pass #収穫モード以外は何もしない


    def storeMotion(self, store):
        """
        格納(いらないかも)
        """

        #[格納]ってそもそも必要なの？
        if self.mode_old == Mode.HERVEST:
            if store:
                self.pic.set_servo_pulsewidth(PIN_SARVO1, CCW_SARVO1)
                self.pic.set_servo_pulsewidth(PIN_SARVO2, CCW_SARVO2)
            else:
                pass #何もしない

            print("store = %s" % store)

        else:
            pass #収穫モード以外は何もしない


    def homeMotion(self, home):
        """
        ホームポジション
        """

        if self.mode_old == Mode.HERVEST:
            if home:
                # ベースを一番上に
                # ...するらしい
                pass #ダミー
            else:
                pass #何もしない

            print("home = %s" % home)

        else:
            pass #収穫モード以外は何もしない


    def tiltMotion(self, tilt):
        """
        手首のチルト
        """

        if self.mode_old == Mode.HERVEST:
            if tilt == Arm.PLUS:
                #手首を上向きに
                if self.tilt_pulse_width >= CW_SARVO2 and DEBUG:
                    self.tilt_pulse_width += 0.1
                else:
                    self.tilt_pulse_width += PLUS_SERVO2
                    if self.tilt_pulse_width > CW_SARVO2:
                        self.tilt_pulse_width = CW_SARVO2
                    else:
                        pass

                if self.tilt_pulse_width > 2500:
                    self.tilt_pulse_width = 2500

            elif tilt == Arm.MINUS:
                #手首を下向きに
                if self.tilt_pulse_width <= CCW_SARVO2 and DEBUG:
                    self.tilt_pulse_width -= 0.1
                else:
                    self.tilt_pulse_width -= PLUS_SERVO2
                    if self.tilt_pulse_width < CCW_SARVO2:
                        self.tilt_pulse_width = CCW_SARVO2
                    else:
                        pass

                if self.tilt_pulse_width < 500:
                    self.tilt_pulse_width = 500

            else:
                pass #止める

            self.pic.set_servo_pulsewidth(PIN_SARVO2, self.tilt_pulse_width)
            print("tilt = %s\t\tWidth = %d" % (tilt, self.tilt_pulse_width))

        else:
            pass #収穫モード以外は何もしない


    def baseMotion(self, updown):
        """
        ベース移動
        """

        if self.mode_old == Mode.HERVEST:
            pwm_duty_cw = SOFTPWM_W_OFF
            pwm_duty_ccw = SOFTPWM_W_OFF

            if updown == Arm.PLUS:
                #ベース位置を上へ
                pwm_duty_ccw = SOFTPWM_W_2_5
            elif updown == Arm.MINUS:
                #ベース位置を下へ
                pwm_duty_cw = SOFTPWM_W_2_5
            else:
                #ベース位置を固定
                pass #何もしない

            self.pic.set_PWM_dutycycle(PIN_INCW, pwm_duty_cw)
            self.pic.set_PWM_dutycycle(PIN_INCCW, pwm_duty_ccw)
            print("updown = %s cw:%d ccw:%d" % (updown, pwm_duty_cw, pwm_duty_ccw))

        else:
            pass #収穫モード以外は何もしない


    def releaseMotion(self, release):
        """
        リリース(いらないと思う)
        """

        if self.mode_old == Mode.HERVEST:
            if release:
                #ベースを一番下に
                #...するのかな？

                #手首を一番上向きに
                self.pic.set_servo_pulsewidth(PIN_SARVO2, CW_SARVO2)

                #離す
                self.pic.set_servo_pulsewidth(PIN_SARVO1, CCW_SARVO1)

            else:
                pass #何もしない

            print("release = %s" % release)
        else:
            pass #収穫モード以外は何もしない

def arm_py():
    """
    アームのメイン
    """

    armc = ArmClass()
    rospy.init_node('arm_py_node', anonymous=True)
    rospy.Subscriber('arm', arm, armc.callback, queue_size=1)
    print("start")
    rospy.spin()

if __name__ == '__main__':
    arm_py()
