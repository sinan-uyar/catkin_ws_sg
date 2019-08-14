#!/usr/bin/env python
import rospy
from autominy_msgs.msg import Speed


def callback(data):
    rospy.loginfo("speed: %f", data.value)
    
def subscriber():

    rospy.init_node('subscriber', anonymous=True)

    rospy.Subscriber("/sensors/speed", Speed, callback)

    rospy.spin()

if __name__ == '__main__':
    subscriber()

