#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

# pythonでROSのソフトウェアを記述するときにimportするモジュール
import rospy

# JoyDummyからの入力用msg
from sensor_msgs.msg import Joy

# Debug on/off
DEBUG = 0

#
HZ_PUBLISH        = 60  #Pubrishの周期
#
STICK_RIGHT_H     = 2   #右スティック水平 
STICK_RIGHT_V     = 5   #右スティック垂直 
STICK_LEFT_H      = 0   #左スティック水平 
STICK_LEFT_V      = 1   #左スティック垂直 
BUTTON_R1         = 18  #R1 
BUTTON_R2         = 4   #R2 
BUTTON_L1         = 17  #L1 
BUTTON_L2         = 3   #L2 
BUTTON_O          = 15  #Oボタン（サークル） 
BUTTON_X          = 14  #xボタン (クロス） 
BUTTON_DELTA      = 16  #△ボタン（デルタ） 
BUTTON_SQUARE     = 13  #□ボタン（スクエア） 
BUTTON_SHARE      = 21  #SHARE 
BUTTON_OPTION     = 22  #OPTION 
CROSS_H           = 9   #十字キー水平 左1 右-1
CROSS_V           = 10  #十字キー垂直 上1 下-1

class JoyDummy(object):
    def __init__(self):
        # 送信作成
        self.pub = rospy.Publisher('joy',Joy , queue_size=100)
        # index
        self.frame_id = 0
    
    def transmit(self):
        msg_joy = Joy()
        with open("/home/ros/arc_osaka/2018/arc_ws/src/tama/scripts/joyDummy.txt") as f:
            s = f.readlines()
        stick_v_l = float(s[0]) 
        stick_v_r =  float(s[1])
        cross_v   =  float(s[2])
        f.close()
        msg_joy.axes = [0,stick_v_l,0,0,0,\
                        stick_v_r,0,0,0,0,\
                        cross_v,0,0]
        msg_joy.buttons = [0,0,0,0,0,\
                        0,0,0,0,0,\
                        0,0,0]
        self.pub.publish(msg_joy)

def joyDummy_py():
    # 初期化宣言 : このソフトウェアは"joyDummy_py_node"という名前
    rospy.init_node('joyDummy_py_node', anonymous=True)
    # インスタンスの作成 
    joyDummy = JoyDummy()
    # 1秒間にpublishする数の設定
    r = rospy.Rate(HZ_PUBLISH)

    # ctl +　Cで終了しない限りwhileループでpublishし続ける
    while not rospy.is_shutdown():
        # publishする関数
        joyDummy.transmit()
        #
        r.sleep()

if __name__ == '__main__':
    try:
            joyDummy_py()

    except rospy.ROSInterruptException: pass

