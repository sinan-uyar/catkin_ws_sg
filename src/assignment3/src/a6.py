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

def pabstand(gerade1, gerade2, punkt1):
    abst=np.linalg.norm(np.cross(np.sub(punkt1, gerade1),gerade2))/np.linalg.norm(gerade2)

    print(str(abst))

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

    #640x480 !!!
    
    #alles ausser Feldmarkierungen schwarz
    cv2.rectangle(cv_image,(0,0),(640,120),(0,0,0),-1)  
    #cv2.rectangle(cv_image,(0,0),(640,480),(0,0,0),-1)  
    cv2.rectangle(cv_image,(200,230),(490,480),(0,0,0),-1) 
    cv2.line(cv_image,(0,250),(320,0),(0,0,0),100)  	

    '''
    counter=0
    for y in range(0,479):
        y=y+1
        for x in range(0,639):
    	    px=cv_image[y,x]
            if px>=200:
                counter=counter+1
            x=x+1

    print(counter)
    '''
    xy1=0
    xy2=0
    while xy1<200:
        x1=random.randint(0,635)
        y1=random.randint(0,475)
        xy1=cv_image[y1,x1]

    while xy2<200:
        x2=random.randint(0,635)
        y2=random.randint(0,475)
        xy2=cv_image[y2,x2]
    '''
    m=(y2-y1)*1.0 / (x1+x2)
    n=(-1*x1)+y1
    mbest=m
    nbest=n
    
    print("f(x)= "+str(m)+"*x+"+str(n))

    #print(m, y1,y2,x1,x2)
    '''
    '''
    for x in range(0,635):
        ye= int(abs(x*m+n))
        #xe=int(xe)
        #if cv_image[y,xe] >=200:
        #    counter+=1
        print(x,ye)
    #if counter>(475*0.5):
    n2=n+10
    n3=n-10
    '''
    
    
    for y in range(0,479):
        for x in range(0,639):
    	    px=cv_image[y,x]
            if px>=200:
                pabst(np.array([x1,y1]),np.array([x2-x1,y2-y1]),np.array([x,y]))
            x=x+1
        y=y+1

    cv2.line(cv_image,(x1,y1),(x2,y2),(255,255,255),20)

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
