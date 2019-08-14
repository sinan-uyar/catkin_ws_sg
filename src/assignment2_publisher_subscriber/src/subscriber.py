#!/usr/bin/env python
import rospy
from autominy_msgs.msg import Speed


def callback(data):
    rospy.loginfo("speed: %f", data.value)
    
def subscriber():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('subscriber', anonymous=True)

    rospy.Subscriber("/sensors/speed", Speed, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    subscriber()


#autominy 
#catkinws
