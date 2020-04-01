#!/usr/bin/env python

import getch
import rospy
from std_msgs.msg import String
from std_msgs.msg import UInt16

def talker():
    pub = rospy.Publisher('command', UInt16, queue_size=10)
    rospy.init_node('commander', anonymous=True)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        k=ord(getch.getch())
        if ( k==114 or k==115) :
            pub.publish(k)
            rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
