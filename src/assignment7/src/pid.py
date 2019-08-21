#!/usr/bin/env python
from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
import random
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#g=vektora+s*(vektor2-vektor1)
def pabstand(gerade1, gerade2, punkt1):
    abst=np.linalg.norm(np.cross(np.subtract(punkt1, gerade1),gerade2))/np.linalg.norm(gerade2)

    return(abst)

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/sensors/camera/infra1/camera_info",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/communication/gps/126",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
    except CvBridgeError as e:
      print(e)

    #640x480 !!!
    
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
