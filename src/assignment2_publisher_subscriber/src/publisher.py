import rospy
from autominy_msgs.msg import SpeedCommand, NormalizedSteeringCommand


class BasicPublisher:

    def __init__(self):
        rospy.init_node("basic_publisher")
        self.speed_publisher = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=1)
        self.steering_publisher = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand,
                                                  queue_size=1)

        self.r = rospy.Rate(10)  # 10hz

        while not rospy.is_shutdown():
            steer_msg = NormalizedSteeringCommand()
            steer_msg.value = 1.0
            self.steering_publisher.publish(steer_msg)

            speed_msg = SpeedCommand()
            speed_msg.value = 0.3
            self.speed_publisher.publish(speed_msg)

            self.r.sleep()


if __name__ == "__main__":
    BasicPublisher()
