#!/usr/bin/env python

import roslib
import rospy
from nav_msgs.msg import Odometry
from autominy_msgs.msg import SpeedCommand, NormalizedSteeringCommand, SteeringCommand, Speed
import tf.transformations
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import String
from std_msgs.msg import Int8
import scipy.interpolate
import numpy as np
import math
#diese bibliothek muss gedownloaded werden !!
from scipy.spatial import distance


class Lane:

    def __init__(self, support_points):
        self.support_points = support_points
        self.spline_x = scipy.interpolate.CubicSpline(self.support_points[:, 0], self.support_points[:, [1]],
                                                      bc_type='periodic')
        self.spline_y = scipy.interpolate.CubicSpline(self.support_points[:, 0], self.support_points[:, [2]],
                                                      bc_type='periodic')

    def length(self):
        return self.support_points[:, 0][-1]

    def interpolate(self, param):
        return np.array([self.spline_x(param)[0], self.spline_y(param)[0]])

    def closest_point(self, point):
        step_size = 0.2
        min_param = 0.0
        max_param = self.length()
        closest_param = -1.0
        closest_distance = float('+inf')
        current_distance = 0.0

        while step_size > 0.001:
            closest_distance = float('+inf')
            closest_param = -1.0
            i = min_param
            while i <= max_param:
                current_distance = np.sum(np.power(self.interpolate(i) - point, 2.0))

                if closest_distance > current_distance:
                    closest_distance = current_distance
                    closest_param = i

                i += step_size

            min_param = max(min_param, closest_param - step_size)
            max_param = min(max_param, closest_param + step_size)
            step_size *= 0.5

        return self.interpolate(closest_param), closest_param

    def lookahead_point(self, point, lookahead_distance):
        closest_point, closest_param = self.closest_point(point)
        dst=closest_param + lookahead_distance
        return self.interpolate(dst), dst


class Map:

    def __init__(self):
        self.lane_1 = np.load("/home/sinanu/lane1.npy")
        self.lane_2 = np.load("/home/sinanu/lane2.npy")
        self.lanes = [
            Lane(self.lane_1[[0, 50, 209, 259, 309, 350, 409, 509, 639, 750, 848, 948, 1028, 1148, 1200, 1276], :]),
            Lane(self.lane_2[[0, 50, 100, 150, 209, 400, 600, 738, 800, 850, 900, 949, 1150, 1300, 1476], :])]


class MapVisualization:

    def __init__(self):
        self.map = Map()
        rospy.init_node("map_visualization")
        self.lane_pub = rospy.Publisher("/lane", Marker, queue_size=10)
        self.clicked_point_subscriber = rospy.Subscriber("/clicked_point", PointStamped, self.on_click, queue_size=1)

        self.rate = rospy.Rate(1)

        while not rospy.is_shutdown():

            i = 0
            for lane in self.map.lanes:
                msg = Marker(type=Marker.LINE_STRIP, action=Marker.ADD)
                msg.header.frame_id = "/map"
                msg.scale.x = 0.01
                msg.scale.y = 0.01
                msg.color.r = 1.0
                msg.color.a = 1.0
                msg.id = i

                for i in range(int(lane.length() * 100.0)):
                    inter = lane.interpolate(i / 100.0)
                    msg.points.append(Point(inter[0], inter[1], 0.0))
                i += 1

                self.lane_pub.publish(msg)

            self.rate.sleep()

    def on_click(self, point_msg):
        i = 2
        point = np.array([point_msg.point.x, point_msg.point.y])
        for lane in self.map.lanes:
            msg = Marker(type=Marker.SPHERE, action=Marker.ADD)
            msg.header.frame_id = "/map"
            msg.scale.x = 0.1
            msg.scale.y = 0.1
            msg.scale.z = 0.1
            msg.color.b = 1.0
            msg.color.a = 1.0
            msg.id = i

            p, param = lane.closest_point(point)
            msg.pose.position.x = p[0]
            msg.pose.position.y = p[1]

            i += 1

            self.lane_pub.publish(msg)

            msg.color.b = 0.0
            msg.color.g = 1.0
            p, param = lane.lookahead_point(point, 0.5)
            msg.pose.position.x = p[0]
            msg.pose.position.y = p[1]
            msg.id = i

            i += 1

            self.lane_pub.publish(msg)



class PID:

    def __init__(self):
        #rospy.init_node("pid")
        self.speed_pub = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=10)
        self.steering_pub = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=10)
        self.yaw_sub = rospy.Subscriber("/yaw", String, self.yaw, queue_size=1)
        self.localization_sub = rospy.Subscriber("/sensors/localization/filtered_map", Odometry, self.on_localization, queue_size=1)
        #self.localization_sub = rospy.Subscriber("/control/steering", SteeringCommand, self.on_steering, queue_size=1)
        print("--------------------------------------------------------------")


        self.pose = Odometry()
        self.rate = rospy.Rate(100)
        self.timer = rospy.Timer(rospy.Duration.from_sec(0.01), self.on_control)

        self.kp = 2.0
        self.ki = 0.0
        self.kd = 0.2
        self.min_i = -1.0
        self.max_i = 1.0

        self.integral_error = 0.0
        self.last_error = 0.0

        # this should be changed from a topic for future tasks
        self.desired_angle = 0

        rospy.on_shutdown(self.on_shutdown)

        while not rospy.is_shutdown():
            self.rate.sleep()

    def on_localization(self, msg):
        self.pose = msg

    def yaw(self, msg):
        self.desired_angle = float(msg.data)
        print(self.desired_angle)

    def on_control(self, tmr):

        if tmr.last_duration is None:
            dt = 0.01
        else:
            dt = (tmr.current_expected - tmr.last_expected).to_sec()

        # print dt
      
        quat = [self.pose.pose.pose.orientation.x, self.pose.pose.pose.orientation.y, self.pose.pose.pose.orientation.z,
                self.pose.pose.pose.orientation.w]
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quat)
        error = self.desired_angle 

        self.integral_error += error * dt
        self.integral_error = max(self.min_i, self.integral_error)
        self.integral_error = min(self.max_i, self.integral_error)

        derivative_error = (error - self.last_error) / dt
        self.last_error = error

        pid_output = self.kp * error + self.kd * derivative_error + self.ki * self.integral_error

        steering_msg = NormalizedSteeringCommand()
        steering_msg.value = pid_output
        self.steering_pub.publish(steering_msg)

    def on_shutdown(self):
        speed_msg = SpeedCommand()
        speed_msg.value = 0.0
        self.speed_pub.publish(speed_msg)

class Speed_controller:

    def __init__(self):
        self.speed_pub = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=10)
        self.yaw_sub = rospy.Subscriber("/sensors/speed", Speed, self.current_speed, queue_size=1)
        self.yaw_sub = rospy.Subscriber("/desired_speed", Int8, self.speed, queue_size=1)


        self.rate = rospy.Rate(100)
        self.timer = rospy.Timer(rospy.Duration.from_sec(0.01), self.on_control)

        self.kp = 2.0
        self.ki = 0.0
        self.kd = 0.2
        self.min_i = -1.0
        self.max_i = 1.0

        self.integral_error = 0.0
        self.last_error = 0.0
        self.curr_speed=0
        # this should be changed from a topic for future tasks
        self.desired_speed = 0

        rospy.on_shutdown(self.on_shutdown)

        while not rospy.is_shutdown():
            self.rate.sleep()

    def current_speed(self, msg):
        self.curr_speed = msg.value

    def speed(self, msg):
        self.desired_speed = msg.value

    def on_control(self, tmr):

        if tmr.last_duration is None:
            dt = 0.01
        else:
            dt = (tmr.current_expected - tmr.last_expected).to_sec()

        error = self.desired_speed - curr_speed

        self.integral_error += error * dt
        self.integral_error = max(self.min_i, self.integral_error)
        self.integral_error = min(self.max_i, self.integral_error)

        derivative_error = (error - self.last_error) / dt
        self.last_error = error

        speed_output = self.kp * error + self.kd * derivative_error + self.ki * self.integral_error
        if speed_output>0.5:
          speed_output=0.5
        if speed_output<0:
          speed_output=0
        speed_msg = Speed()
        speed_msg.value = speed_output
        self.speed_pub.publish(speed_msg)

    def on_shutdown(self):
        speed_msg = SpeedCommand()
        speed_msg.value = 0.0
        self.speed_pub.publish(speed_msg)

class Drive_controll:

    def __init__(self):
        rospy.init_node("drive_controll")
        self.position_sub = rospy.Subscriber("/sensors/localization/filtered_map", Odometry, self.calculate_yaw, queue_size=1)
        self.change_lane_sub = rospy.Subscriber("/change_lane", Int8 , self.lane_change, queue_size=1)
        self.yaw_pub = rospy.Publisher("/yaw", String, queue_size=10)
        self.steering_pub = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=10)
        self.current_lane=0
        self.map=Map()
        self.pid=PID()
        #self.steering_pub = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=10)

    def lane_change(self, data):
      
      if data.data==0:
        self.current_lane=0
      else:
        self.current_lane=1

    def calculate_yaw(self, data):
     
      curr_point = np.array([data.pose.pose.position.x, data.pose.pose.position.y])
      la_point,dst= self.map.lanes[self.current_lane].lookahead_point(curr_point, 0.5)

      quat = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
      roll, pitch, current_yaw = tf.transformations.euler_from_quaternion(quat)

      r=np.array([ [np.cos(-current_yaw), -np.sin(-current_yaw)],
                   [np.sin(-current_yaw), np.cos(-current_yaw)] ])
      v= la_point - curr_point
      v_map=np.matmul(r,v)
      #print("")
      #print("")
      #print(v)
      #print(r)
      #print(v_map)
      yaw_value=math.atan2(v_map[1],v_map[0]) 
      print(yaw_value)
      print(" ")
      f=String()
      f.data=str(yaw_value)	
      self.yaw_pub.publish(f)

if __name__ == "__main__":
    Drive_controll()
    try:
      rospy.spin()
    except KeyboardInterrupt:
      print("Shutting down")

