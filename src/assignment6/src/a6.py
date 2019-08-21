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
    abst=np.linalg.norm(np.cross(np.subtract(punkt1, gerade1),gerade2))/np.linalg.norm(gerade2)

    return(abst)

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
    
    ret,th1=cv2.threshold(cv_image, 200, 255, cv2.THRESH_BINARY )
    cv2.rectangle(th1,(0,0),(640,120),(0,0,0),-1)  
    #cv2.rectangle(cv_image,(0,0),(640,480),(0,0,0),-1)  
    cv2.rectangle(th1,(200,230),(490,480),(0,0,0),-1) 
    cv2.line(th1,(0,250),(320,0),(0,0,0),100)  	

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
    
    print()
    x7=0
    while x7<3:
	    x6=0
	    x1best=0
	    y1best=0
	    x2best=0
	    y2best=0
	    bestcounter=0
	    while x6<8:
		    xy1=0
		    xy2=0
		    while xy1<200:
			x1=random.randint(0,635)
			y1=random.randint(0,475)
			xy1=th1[y1,x1]

		    while xy2<200:
			x2=random.randint(0,635)
			y2=random.randint(0,475)
			xy2=th1[y2,x2]

		    counter=0
		    for y in range(0,479):
			for x in range(0,639):
		    	    px=th1[y,x]
		            x4=x
		            y4=y
			    if px>=200:
				erg=pabstand(np.array([x1,y1]),np.array([x2-x1,y2-y1]),np.array([x4,y4]))
				if erg<5:
		                    #cv2.line( cv_image,(x,y),(x,y),(0,0,0) )
				    counter+=1
				    #print(x,y,erg)
			    x=x+1
			y=y+1
		    x6+=1
		    if counter>bestcounter:
		        bestcounter=counter
		        x1best=x1
			y1best=y1
			x2best=x2
			y2best=y2
	    
	    m=(y2best-y1best)*1.0 / (x2best-x1best)
	    n=y1best-m*x1best
	    
	    cv2.line(th1,(0,int(n)),(640,int(m*640+n)),(100,100,100),30)
            x7+=1
    	    print(str(x7)+". line: "+"m= "+str(m)+" n= "+str(n))


    #cv2.imshow("Image window", th1)
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
