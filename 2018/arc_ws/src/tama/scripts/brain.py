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
from param import Mode

# Debug on/off
DEBUG = 0

#
HZ_PUBLISH        = 60  #Pubrishの周期
HZ_MODE           = 5
SPEED_STEP        = 64
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
INDEX_SPEED_L     = STICK_LEFT_V 
INDEX_SPEED_R     = STICK_RIGHT_V  
INDEX_STRIKE_ON   = BUTTON_O
INDEX_STRIKE_OFF  = BUTTON_X
INDEX_GRUB_ON     = BUTTON_O 
INDEX_GRUB_OFF    = BUTTON_X
INDEX_HOME        = BUTTON_DELTA 
INDEX_STORE       = BUTTON_L1
INDEX_TILT        = CROSS_V 
INDEX_UP          = BUTTON_R1
INDEX_DOWN        = BUTTON_R2 
INDEX_RELEASE     = BUTTON_SHARE
INDEX_MODE        = BUTTON_OPTION
INDEX_MAX         = BUTTON_OPTION
#
BOADER_DIRECTION  = 0.2
BOADER_SPEED      = 0.5
BOADER_GRUB       = -0.01
BOADER_TILT       = 0.2
#
MAX = 1
MIN = -1

#
ON  = 1
OFF = 0 

#
PIN_MICRO_SW_UP =   5#6
PIN_MICRO_SW_DOWN = 18#27

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
        # updownの衝突判定用
        self.pi = pigpio.pi()
        self.pi.set_mode(PIN_MICRO_SW_UP,pigpio.INPUT)
        self.pi.set_mode(PIN_MICRO_SW_DOWN,pigpio.INPUT)
        #
        self.mode = Mode.HERVEST
        self.strike = False
        self.grub = False
        self.cnt_mode = 0

    def clearMsg(self):
        #arm
        self.msg_arm.strike = False
        self.msg_arm.home   = False
        self.msg_arm.release= False
        self.msg_arm.grub   = False
        self.msg_arm.store  = False
        self.msg_arm.tilt   = Arm.NONE
        self.msg_arm.updown = Arm.NONE
        self.msg_arm.mode   = Mode.HERVEST
        # foot
        self.msg_foot.direction_l = Direction.AHEAD
        self.msg_foot.direction_r = Direction.AHEAD
        self.msg_foot.speed_l     = 0
        self.msg_foot.speed_r     = 0

    def convertMode(self):
        if self.operation[INDEX_MODE] == 1:
            if (self.cnt_mode % ( HZ_PUBLISH/HZ_MODE )) == 0:
                self.cnt_mode = 0
                if self.mode == Mode.HERVEST :
                    self.mode = Mode.BULB
                else:
                    self.mode = Mode.HERVEST
        self.msg_arm.mode = self.mode
        self.cnt_mode += 1

    def convertUpDown(self):
        micro_sw_up   = self.pi.read(PIN_MICRO_SW_UP)
        micro_sw_down = self.pi.read(PIN_MICRO_SW_DOWN)
        # oparating
        if self.operation[INDEX_UP] == 1 and micro_sw_up == 1:  
            self.msg_arm.updown = Arm.PLUS 
        elif self.operation[INDEX_DOWN] != -1 and micro_sw_down == 1:  
            self.msg_arm.updown = Arm.MINUS
            
    def convertFoot(self):
        # foot
        ## direction
        if self.operation[INDEX_SPEED_L] >= 0:  
            self.msg_foot.direction_l = Direction.AHEAD
        else:  
            self.msg_foot.direction_l = Direction.BACK
        if self.operation[INDEX_SPEED_R] >= 0:  
            self.msg_foot.direction_r = Direction.AHEAD
        else:  
            self.msg_foot.direction_r = Direction.BACK
        ## speed
        speed = self.operation[INDEX_SPEED_L]  
        self.msg_foot.speed_l = int(speed*SPEED_STEP*100)/SPEED_STEP
        speed = self.operation[INDEX_SPEED_R]  
        self.msg_foot.speed_r = int(speed*SPEED_STEP*100)/SPEED_STEP
       
    def printMsg(self):
        print "UpDown=" + str(self.msg_arm.updown)
        print "Dir_L =" + str(self.msg_foot.direction_l)
        print "Dir_R =" + str(self.msg_foot.direction_r)
        print "Spd_L =" + str(self.msg_foot.speed_l)
        print "Spd_R =" + str(self.msg_foot.speed_r)
        
    def convert(self):
        #print(self.operation)
        # arm
        if self.operation[INDEX_STRIKE_ON] == 1:
            self.strike = True
        if self.operation[INDEX_STRIKE_OFF] == 1:
            self.strike = False
        self.msg_arm.strike = self.strike
        self.msg_arm.home = bool(self.operation[INDEX_HOME])
        self.msg_arm.release= bool(self.operation[INDEX_RELEASE])
        ## grub & store
        if self.operation[INDEX_GRUB_ON] == 1:
            self.grub = True
        if self.operation[INDEX_GRUB_OFF] == 1:
            self.grub = False
        self.msg_arm.grub = self.grub 
        self.msg_arm.store = bool(self.operation[INDEX_STORE])
        ## tilt
        if self.operation[INDEX_TILT] == 1:  
            self.msg_arm.tilt = Arm.PLUS 
        if self.operation[INDEX_TILT] == -1:  
            self.msg_arm.tilt= Arm.MINUS
        ## updown
        self.convertUpDown()
        # mode
        self.convertMode()
        
        # foot
        self.convertFoot()
         
    def transmit(self):
        # clear
        self.clearMsg()
        self.msg_arm.frame_id = self.frame_id
        self.msg_foot.frame_id = self.frame_id
        if len(self.operation) > INDEX_MAX:
            self.convert()
        # publishする関数
        self.pub_arm.publish(self.msg_arm)
        self.pub_foot.publish(self.msg_foot)
        self.frame_id+=1

        #self.printMsg()

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
    r = rospy.Rate(HZ_PUBLISH)

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

