#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool

def callback(msg):
    pub = rospy.Publisher('/walker_active', Bool, queue_size=1, latch=True)
    if msg.data:
        rospy.loginfo("Ball detected, stopping walker.")
        pub.publish(Bool(data=False))
    else:
        pub.publish(Bool(data=True))

def controller():
    rospy.init_node('main_controller')
    rospy.Subscriber('/target_detected', Bool, callback)
    rospy.spin()

if __name__ == '__main__':
    controller()
