#!/usr/bin/env python
from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/sensors/camera/infra1/camera_info",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/sensors/camera/infra1/image_rect_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
    except CvBridgeError as e:
      print(e)

    #(rows,cols,channels) = cv_image.shape
    #640x480 !!!
    #if cols > 60 and rows > 60 :
    #  cv2.circle(cv_image, (50,50), 10, 255)
    #cv2.line(cv_image,(0,100),(400,0),(0,0,0),150)  
    cv2.rectangle(cv_image,(0,0),(640,100),(0,0,0),-1)  
    cv2.rectangle(cv_image,(0,0),(220,150),(0,0,0),-1)  
    cv2.rectangle(cv_image,(250,230),(460,350),(0,0,0),-1) 
    cv2.rectangle(cv_image,(0,350),(640,480),(0,0,0),-1)  

    '''
    cv2.line(cv_image,(0,200),(640,200),(255,255,255))  
    cv2.line(cv_image,(0,400),(640,400),(255,255,255),1)  
    cv2.line(cv_image,(200,0),(200,480),(255,255,255),1)  
    cv2.line(cv_image,(400,0),(400,480),(255,255,255),1)  
    cv2.line(cv_image,(600,0),(600,480),(255,255,255),1)
    '''

    '''
    for y in range(480):
        y=y+1
        for x in range(640):
    	    px=cv_image[y,x]
            
            rospy.loginfo("x: "+str(x)+" y: "+str(y)+"	"+str(px))
	    if 
            x=x+1
    '''
    ret,th1=cv2.threshold(cv_image, 200, 255, cv2.THRESH_BINARY )

    cv2.imshow("Image window", th1)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(th1, "mono8"))
    except CvBridgeError as e:
      print(e)

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
