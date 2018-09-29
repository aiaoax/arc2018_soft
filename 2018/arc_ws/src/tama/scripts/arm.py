#!/usr/bin/env python
"""
Reference:https://karaage.hatenadiary.jp/entry/2017/02/10/073000
Summery: Move servo angle to the topic value 'servo_angle'
"""

import pigpio
import rospy
from tama.msg import arm

# initialize gpio
pwm_pin = 18
pi = pigpio.pi()
pi.set_mode(pwm_pin, pigpio.OUTPUT)

def callback(arm):
    duty = ((arm.frameID % 90.) / 180. * 1.9 % 0.5)\
            / 20. * 1e6
    pi.hardware_PWM(pwm_pin, 50, 50000)
    print ('frameID = %d ' % arm.frameID )
    print ("strike = %s" %  arm.strike ) 
    print("grub = %s" % arm.grub )
    print("store = %s" % arm.store )
    print("home = %s" % arm.home )
    print("tilt = %s" % arm.tilt )
    print("updown = %s" % arm.updown )
    print("release = %s" % arm.release) 
    print("=============") 
        
def arm_py():
    rospy.init_node('arm_py_node',anonymous=True)
    sub=rospy.Subscriber('arm', arm, callback, queue_size=1)
    print "start"
    rospy.spin()

if __name__ == '__main__':
   arm_py()
