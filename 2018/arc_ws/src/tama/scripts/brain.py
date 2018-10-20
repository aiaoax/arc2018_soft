#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

# pythonでROSのソフトウェアを記述するときにimportするモジュール
import rospy

# gpio制御用
import pigpio

# 自分で定義したmessageファイルから生成されたモジュール
from tama.msg import arm
from tama.msg import foot 
# JoyStickControllerからの入力用msg
from sensor_msgs.msg import Joy

# 定数などの定義ファイルimport
from param import Direction
from param import Speed
from param import Arm

# RotaryEncoder用
from rotaryEncoder import RotaryEncoder

# Debug on/off
DEBUG = 0

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
#
MAX = 1
MIN = -1

#
PIN_ROTARY_A = 5
PIN_ROTARY_B = 6
PIN_MICRO_SW = 18#27
#
IN_INITIAL = 0
IN_OPERATE = 1

MAX_ROTATE = 0
MIN_ROTATE = 300

class Brain(object):
    def __init__(self):
        self.operation = []
        # 受信作成
        self.sub = rospy.Subscriber('joy', Joy, self.joyCallback, queue_size=1)
        # 送信作成
        self.pub_arm = rospy.Publisher('arm', arm, queue_size=100)
        self.pub_foot = rospy.Publisher('foot',foot, queue_size=100)
        # messageのインスタンスを作る
        self.msg_arm = arm()
        self.msg_foot = foot()
        # index
        self.frame_id = 0
        # rotaryencoder
        self.rotary = RotaryEncoder()
        self.rotary.setPin(PIN_ROTARY_A,PIN_ROTARY_B)
        # updownの衝突判定用
        self.updown = IN_INITIAL
        self.pi = pigpio.pi()
        self.pi.set_mode(PIN_MICRO_SW,pigpio.INPUT)

    def clearMsg(self):
        #arm
        self.msg_arm.strike = False
        self.msg_arm.home   = False
        self.msg_arm.release= False
        self.msg_arm.grub   = False
        self.msg_arm.store  = False
        self.msg_arm.tilt   = Arm.NONE
        self.msg_arm.updown = Arm.NONE
        # foot
        self.msg_foot.direction = Direction.STOP
        self.msg_foot.speed     = Speed.LOW

    def convertUpDown(self):
        # initialize
        if  self.updown == IN_INITIAL:
            micro_sw = self.pi.read(PIN_MICRO_SW)
            if micro_sw == 1 :
                self.updown = 0
                self.rotary.setRotate(0)
            else:
                self.msg_arm.updown = Arm.PLUS
        # oparating
        if self.updown == IN_OPERATE:
            rotate = self.rotary.getRotate()
            if self.operation[INDEX_UPDOWN] == MAX and rotate > MIN_ROTATE:  
                self.msg_arm.updown = Arm.PLUS 
            if self.operation[INDEX_UPDOWN] == MIN and rotate < MAX_ROTATE:  
                self.msg_arm.updown = Arm.MINUS
            
        print "rotate " + str(self.rotary.getRotate())

    def convert(self):
        print(self.operation)
        # arm
        self.msg_arm.strike = bool(self.operation[INDEX_STRIKE])
        self.msg_arm.home = bool(self.operation[INDEX_HOME])
        self.msg_arm.release= bool(self.operation[INDEX_RELEASE])
        ## grub & store
        if self.operation[INDEX_GRUB] < BOADER_GRUB:  
            self.msg_arm.grub = True 
            self.msg_arm.store = bool(self.operation[INDEX_STORE])
        ## tilt
        if self.operation[INDEX_TILT] > BOADER_TILT:  
            self.msg_arm.tilt = Arm.PLUS 
        if self.operation[INDEX_TILT] < -1*BOADER_TILT:  
            self.msg_arm.tilt= Arm.MINUS
        ## updown
        self.convertUpDown()

        # foot
        ## direction
        if self.operation[INDEX_DIRECTION_V] > BOADER_DIRECTION:  
            self.msg_foot.direction = Direction.AHEAD
        elif self.operation[INDEX_DIRECTION_V] < -1*BOADER_DIRECTION:  
            self.msg_foot.direction = Direction.BACK
        elif self.operation[INDEX_DIRECTION_H] > BOADER_DIRECTION:  
            self.msg_foot.direction = Direction.LEFT
        elif self.operation[INDEX_DIRECTION_H] < -1*BOADER_DIRECTION:  
            self.msg_foot.direction = Direction.RIGHT
        ## speed
        if self.operation[INDEX_SPEED] == MIN:  
            self.msg_foot.speed = Speed.HIGH
        elif self.operation[INDEX_SPEED] < BOADER_SPEED:  
            self.msg_foot.speed = Speed.MIDDLE

    def transmit(self):
        # clear
        self.clearMsg()
        self.msg_arm.frame_id = self.frame_id;
        self.msg_foot.frame_id = self.frame_id;
        if len(self.operation) > INDEX_MAX:
            self.convert()
        # publishする関数
        self.pub_arm.publish(self.msg_arm)
        self.pub_foot.publish(self.msg_foot)
        self.frame_id+=1
        print "frame_id" + str(self.msg_foot.frame_id)

        print "rotate " + str(self.rotary.getRotate())
    def joyCallback(self, joy_msg):
        j = 0
        # ２回以降は代入
        if len(self.operation) > INDEX_MAX:
            for i,item in enumerate(joy_msg.axes):
                self.operation[i] = item
                j=i
            for i,item in enumerate(joy_msg.buttons):
                self.operation[j+i] = item
        # 初回は追加
        else:
            for i,item in enumerate(joy_msg.axes):
                self.operation.append(item)
                j=i
            for i,item in enumerate(joy_msg.buttons):
                self.operation.append(item)
        # for debug            
        if(DEBUG):
            for i ,item in enumerate(self.operation):
                print str(i) + "=" + str(self.operation[i])

            
def brain_py():
    # 初期化宣言 : このソフトウェアは"brain_py_node"という名前
    rospy.init_node('brain_py_node', anonymous=True)
    # インスタンスの作成 
    brain = Brain()
    # 1秒間にpublishする数の設定
    r = rospy.Rate(5)

    print"start brain"
    # ctl +　Cで終了しない限りwhileループでpublishし続ける
    while not rospy.is_shutdown():
        # publishする関数
        brain.transmit()
        #
        r.sleep()

if __name__ == '__main__':
    try:
            brain_py()

    except rospy.ROSInterruptException: pass

