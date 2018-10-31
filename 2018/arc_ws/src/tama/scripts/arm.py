#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=E1101,C0103,C0325
"""
アーム
"""

import pigpio
import rospy

from tama.msg import arm
from param import Arm
from param import Mode

from arm_consts import\
                PIN_INCCW, PIN_INCW, PIN_SARVO1, PIN_SARVO2, PLUS_SERVO2,\
                SOFTPWM_F_20K, SOFTPWM_W_2_5, SOFTPWM_W_OFF,\
                CCW_SARVO1, CCW_SARVO2, CW_SARVO1, CW_SARVO2,\
                BASE_DN_FIGURE, BASE_UP_FIGURE,\
                DEBUG

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
        self.mode_old = Mode.BOOT

        self.modeChange(Mode.BOOT)


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
                mode_file = open('/usr/local/www/mode.txt', 'w')
                tmp_str = "UNKNOWN"

                #モード名を取得
                tmp_str = Mode(mode).name

                mode_file.write(tmp_str)

            finally:
                mode_file.close()

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

        if self.mode_old > 0:
            print("mode = %s" % Mode(self.mode_old).name)
        else:
            print("mode = %s" % "UNKNOWN")


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
            print("strike = %s\t\tPWM = %f" % (strike, pwm_duty))

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
            print("tilt = %s\t\tWidth = %d" % (Arm(tilt).name, self.tilt_pulse_width))

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
                pwm_duty_ccw = SOFTPWM_W_2_5 * BASE_UP_FIGURE
            elif updown == Arm.MINUS:
                #ベース位置を下へ
                pwm_duty_cw = SOFTPWM_W_2_5 * BASE_DN_FIGURE
            else:
                #ベース位置を固定
                pass #何もしない

            self.pic.set_PWM_dutycycle(PIN_INCW, pwm_duty_cw)
            self.pic.set_PWM_dutycycle(PIN_INCCW, pwm_duty_ccw)
            print("updown = %s\t\tcw:%d ccw:%d" % (Arm(updown).name, pwm_duty_cw, pwm_duty_ccw))

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
