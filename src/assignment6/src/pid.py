#!/usr/bin/env python
from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
import tf
import math
from nav_msgs.msg import Odometry
from autominy_msgs.msg import NormalizedSteeringCommand
from autominy_msgs.msg import SpeedCommand
def nothing(x):
    pass

img = np.zeros((300,512,3), np.uint8)
cv2.namedWindow('image')
cv2.createTrackbar('kp','image',0,20,nothing)
cv2.createTrackbar('ki','image',0,100,nothing)
cv2.createTrackbar('kd','image',0,100,nothing)
cv2.createTrackbar('speed','image',-50,50,nothing)


class image_converter: 
 

  def __init__(self):
    self.error_acc=0
    self.time_last=rospy.Time.now()
    self.time_difference=0
    self.x2 = rospy.Subscriber("/communication/gps/6",Odometry,self.callback)
    self.x3 = rospy.Publisher("/actuators/steering_normalized",NormalizedSteeringCommand)
    self.x4 = rospy.Publisher("/actuators/speed",SpeedCommand)
    print("init")

  def pid(self,yaw):
    error=0-yaw
    self.error_acc+=error
    time_current=rospy.Time.now()
    time_difference=(time_current-self.time_last).to_sec()
    print(self.time_last)
    time_last=time_current
    #kp=cv2.getTrackbarPos('kp','image')
    kp=0
    #ki=cv2.getTrackbarPos('ki','image')
    ki=0
    #kd=cv2.getTrackbarPos('kd','image')
    kd=17
    u=kp*error+kd*(error/time_difference)+ ki*self.error_acc*time_difference
    #print(u)
    return u

  def callback(self,raw_msgs):
    #print("call")
    msg=raw_msgs.pose.pose.orientation
    msg_array=[msg.x,msg.y,msg.z,msg.w]
    #print(str(msg))
    euler=tf.transformations.euler_from_quaternion(msg_array)
    #print(euler)
    steering=NormalizedSteeringCommand()
    steering.value=self.pid(euler[2])
    speed=SpeedCommand()
    speed.value=0.5
    #speed.value=cv2.getTrackbarPos('speed','image')/100
    #print(str(steering.value))
    cv2.imshow('image',img)
    cv2.waitKey(3)
    self.x4.publish(speed)

    try:
      self.x3.publish(steering)
    except CvBridgeError as e:
      print(e)


#steering normalized
def main():
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
