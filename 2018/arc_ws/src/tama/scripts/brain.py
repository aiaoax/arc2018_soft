#!/usr/bin/python
# -*- coding: utf-8 -*-
# license removed for brevity

# pythonでROSのソフトウェアを記述するときにimportするモジュール
import rospy

# 自分で定義したmessageファイルから生成されたモジュール
from tama.msg import arm
from tama.msg import foot 

#定数などの定義ファイルimport
from param import Direction
from param import Speed

def brain_py():
    # 初期化宣言 : このソフトウェアは"para_in"という名前
    rospy.init_node('brain_py_node', anonymous=True)

    # nodeの宣言 : publisherのインスタンスを作る
    # input_dataというtopicにAdder型のmessageを送るPublisherをつくった
    pub_arm = rospy.Publisher('arm', arm, queue_size=100)
    pub_foot = rospy.Publisher('foot',foot, queue_size=100)

    # 1秒間にpublishする数の設定
    r = rospy.Rate(5)

    frame_id = 0

    # Adder型のmessageのインスタンスを作る
    msg_arm = arm()
    msg_foot = foot()
    
    print"start brain"
    # ctl +　Cで終了しない限りwhileループでpublishし続ける
    
    while not rospy.is_shutdown():

        msg_arm.frame_id = frame_id
        msg_foot.frame_id = frame_id
        msg_foot.direction = int(Direction.RIGHT)
        msg_foot.speed = int(Speed.LOW)
        
        # publishする関数
        pub_arm.publish(msg_arm)
        pub_foot.publish(msg_foot)
        frame_id += 1

        r.sleep()

if __name__ == '__main__':
    try:
            brain_py()

    except rospy.ROSInterruptException: pass

