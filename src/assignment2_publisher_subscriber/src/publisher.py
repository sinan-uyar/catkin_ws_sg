#/usr/bin/env python
# license removed for brevity
import rospy

def publisher():
    pub = rospy.Publisher('/actuators/steering', autominy_msgs/NormalizedSteeringCommand, queue_size=10)
    pub2 = rospy.Publisher('/actuators/speed', autominy_msgs/SpeedCommand, queue_size=10)
    rospy.init_node('publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(1.0)
	pub2.publish(0.3)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

