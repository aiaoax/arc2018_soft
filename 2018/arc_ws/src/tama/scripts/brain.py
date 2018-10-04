#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

# pythonでROSのソフトウェアを記述するときにimportするモジュール
import rospy

# 自分で定義したmessageファイルから生成されたモジュール
from tama.msg import arm
from tama.msg import foot 

from sensor_msgs.msg import Joy

#定数などの定義ファイルimport
from param import Direction
from param import Speed
from param import Arm

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
BUTTON_SHARE      = 21  #SHARE 
CROSS_H           = 9   #十字キー水平 
CROSS_V           = 10  #十字キー垂直 

#
INDEX_DIRECTION_H = STICK_RIGHT_H  
INDEX_DIRECTION_V = STICK_RIGHT_V 
INDEX_SPEED       = BUTTON_R2 #neutoral=1,Max=-1
INDEX_STRIKE      = BUTTON_R1
INDEX_GRUB        = BUTTON_L2 #neutoral=1,Max=-1
INDEX_HOME        = BUTTON_O 
INDEX_STORE       = BUTTON_L1
INDEX_TILT        = STICK_LEFT_V    
INDEX_UPDOWN      = CROSS_V
INDEX_RELEASE     = BUTTON_SHARE
INDEX_MAX         = BUTTON_SHARE
#
BOADER_DIRECTION  = 0.2
BOADER_SPEED      = 0.5
BOADER_GRUB       = -0.01
BOADER_TILT       = 0.2

class Brain(object):
    def __init__(self):
        self.operation = []
        self.sub = rospy.Subscriber('joy', Joy, self.joyCallback, queue_size=1)
        self.pub_arm = rospy.Publisher('arm', arm, queue_size=100)
        self.pub_foot = rospy.Publisher('foot',foot, queue_size=100)
        # messageのインスタンスを作る
        self.msg_arm = arm()
        self.msg_foot = foot()

    def initializeMsg(self):
        self.msg_arm.strike = False
        self.msg_arm.grub   = False
        self.msg_arm.store  = False
        self.msg_arm.home   = False
        self.msg_arm.tilt   = Arm.NONE
        self.msg_arm.updown = Arm.NONE
        self.msg_arm.release= False

        self.msg_foot.direction = Direction.STOP
        self.msg_foot.speed     = Speed.LOW

    def convert(self):
        #arm
        self.msg_arm.strike = bool(self.operation[INDEX_STRIKE])
        if self.operation[INDEX_GRUB] < BOADER_GRUB:  
            self.msg_arm.grub = True 
            self.msg_arm.store = bool(self.operation[INDEX_STORE])
        self.msg_arm.home = bool(self.operation[INDEX_HOME])
        if self.operation[INDEX_TILT] > BOADER_TILT:  
            self.msg_arm.tilt = Arm.PLUS 
        if self.operation[INDEX_TILT] < -1*BOADER_TILT:  
            self.msg_arm.tilt= Arm.MINUS
        if self.operation[INDEX_UPDOWN] == 1:  
            self.msg_arm.updown = Arm.PLUS 
        if self.operation[INDEX_UPDOWN] == -1:  
            self.msg_arm.updown = Arm.MINUS
        self.msg_arm.release= bool(self.operation[INDEX_RELEASE])
        #foot
        if self.operation[INDEX_DIRECTION_V] > BOADER_DIRECTION:  
            self.msg_foot.direction = Direction.AHEAD
        elif self.operation[INDEX_DIRECTION_V] < -1*BOADER_DIRECTION:  
            self.msg_foot.direction = Direction.BACK
        elif self.operation[INDEX_DIRECTION_H] > BOADER_DIRECTION:  
            self.msg_foot.direction = Direction.LEFT
        elif self.operation[INDEX_DIRECTION_H] < -1*BOADER_DIRECTION:  
            self.msg_foot.direction = Direction.RIGHT
        if self.operation[INDEX_SPEED] == -1:  
            self.msg_foot.speed = Speed.HIGH
        elif self.operation[INDEX_SPEED] < BOADER_SPEED:  
            self.msg_foot.speed = Speed.MIDDLE

    def transmit(self):
        #clear
        self.initializeMsg()
        if len(self.operation) > INDEX_MAX:
            self.convert()
        # publishする関数
        print "strike "+ str(self.msg_arm.strike)
        self.pub_arm.publish(self.msg_arm)
        self.pub_foot.publish(self.msg_foot)

        print "length" + str(self.msg_arm.strike)
    def joyCallback(self, joy_msg):
        j = 0
        if len(self.operation) > INDEX_MAX:
            for i,item in enumerate(joy_msg.axes):
                self.operation[i] = item
                j=i
            for i,item in enumerate(joy_msg.buttons):
                self.operation[j+i] = item
        else:
            for i,item in enumerate(joy_msg.axes):
                self.operation.append(item)
                j=i
            for i,item in enumerate(joy_msg.buttons):
                self.operation.append(item)
#for debug            
        for i ,item in enumerate(self.operation):
            print str(i) + "=" + str(self.operation[i])


def brain_py():
    # 初期化宣言 : このソフトウェアは"para_in"という名前
    rospy.init_node('brain_py_node', anonymous=True)

    # 
    brain = Brain()

    # 1秒間にpublishする数の設定
    r = rospy.Rate(5)

    frame_id = 0
    
    print"start brain"
    # ctl +　Cで終了しない限りwhileループでpublishし続ける
    while not rospy.is_shutdown():
        brain.msg_arm.frame_id = frame_id
        brain.msg_foot.frame_id = frame_id
        # publishする関数
        brain.transmit()

        frame_id += 1

        r.sleep()

if __name__ == '__main__':
    try:
            brain_py()

    except rospy.ROSInterruptException: pass

