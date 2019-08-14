#!/usr/bin/env python


# publisher according to talker chatter example from tutorial
# publishes steer value 1.0 and speed value 0.3 m/s wit frequency of 10hz
import rospy
import autominy_msgs

from autominy_msgs.msg import NormalizedSteeringCommand
from autominy_msgs.msg import SpeedCommand

def publi():
    pub = rospy.Publisher('/actuators/steering', NormalizedSteeringCommand, queue_size=10)
    pub2 = rospy.Publisher('/actuators/speed', SpeedCommand, queue_size=10)
    rospy.init_node('publi', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(NormalizedSteeringCommand, 1.0)
	pub2.publish(SpeedCommand, 0.3)
        rate.sleep()

if __name__ == '__main__':
    try:
        publi()
    except rospy.ROSInterruptException:
        pass

