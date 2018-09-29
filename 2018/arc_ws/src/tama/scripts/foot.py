#!/usr/bin/env python
"""
Reference:https://karaage.hatenadiary.jp/entry/2017/02/10/073000
Summery: Move servo angle to the topic value 'servo_angle'
"""

import pigpio
import rospy
from tama.msg import foot

# initialize gpio
pwm_pin = 18
pi = pigpio.pi()
pi.set_mode(pwm_pin, pigpio.OUTPUT)

def callback(foot):
    duty = ((foot.frameID + 90.) / 180. * 1.9 + 0.5)\
            / 20. * 1e6
    pi.hardware_PWM(pwm_pin, 50, int(0))
    print("frameID = %d" % foot.frameID)
    print("direction = %d" % foot.direction)
    print("speed = %d" % foot.speed)
    print"==============="

def foot_py():
    rospy.init_node('foot_py_node',anonymous=True)
    sub=rospy.Subscriber('foot', foot, callback, queue_size=1)
    print "start"
    rospy.spin()

if __name__ == '__main__':
   foot_py()
