#!/usr/bin/env python
from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
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

    #640x480 !!!
    
    #alles ausser Feldmarkierungen schwarz
    cv2.rectangle(cv_image,(0,0),(640,100),(0,0,0),-1)  
    cv2.rectangle(cv_image,(0,0),(220,150),(0,0,0),-1)  
    cv2.rectangle(cv_image,(250,230),(460,350),(0,0,0),-1) 
    cv2.rectangle(cv_image,(0,350),(640,480),(0,0,0),-1)  	

    #in 6 bereiche einteilen (rechtecke um Markierungen, um werte fuer bereiche zu erhalten)
    cv2.rectangle(cv_image,(256,110),(279,133),(255,255,255)) 
    cv2.rectangle(cv_image,(407,102),(427,122),(255,255,255))  
	
    cv2.rectangle(cv_image,(236,150),(259,175),(255,255,255)) 
    cv2.rectangle(cv_image,(433,140),(457,162),(255,255,255))  

    cv2.rectangle(cv_image,(190,227),(222,252),(255,255,255)) 
    cv2.rectangle(cv_image,(485,210),(520,239),(255,255,255)) 


    ret,th1=cv2.threshold(cv_image, 200, 255, cv2.THRESH_BINARY )

    #xsum11 ist x avrg von top left, xsum32 ist x avrg von bottom right
    #fuer jeden bereich alle pixel durchgehen und deren durchschnittliche x- und y-Koordinate herausfinden

    xsum11=0
    ysum11=0
    counter=0
    for y in range(111,133):
        y=y+1
        for x in range(257,279):
    	    px=cv_image[y,x]
            if px>=200:
                xsum11=xsum11+x
                ysum11=ysum11+y
                counter=counter+1
            x=x+1
    xsum11=xsum11//counter
    ysum11=ysum11//counter

    xsum12=0
    ysum12=0
    counter=0
    for y in range(103,122):
        y=y+1
        for x in range(408,427):
    	    px=cv_image[y,x]
            if px>=200:
                xsum12=xsum12+x
                ysum12=ysum12+y
                counter=counter+1
            x=x+1
    xsum12=xsum12//counter
    ysum12=ysum12//counter

    xsum21=0
    ysum21=0
    counter=0
    for y in range(151,175):
        y=y+1
        for x in range(237,259):
    	    px=cv_image[y,x]
            if px>=200:
                xsum21=xsum21+x
                ysum21=ysum21+y
                counter=counter+1
            x=x+1
    xsum21=xsum21//counter
    ysum21=ysum21//counter

    xsum22=0
    ysum22=0
    counter=0
    for y in range(141,162):
        y=y+1
        for x in range(434,457):
    	    px=cv_image[y,x]
            if px>=200:
                xsum22=xsum22+x
                ysum22=ysum22+y
                counter=counter+1
            x=x+1
    xsum22=xsum22//counter
    ysum22=ysum22//counter

    xsum31=0
    ysum31=0
    counter=0
    for y in range(228,252):
        y=y+1
        for x in range(191,222):
    	    px=cv_image[y,x]
            if px>=200:
                xsum31=xsum31+x
                ysum31=ysum31+y
                counter=counter+1
            x=x+1
    xsum31=xsum31//counter
    ysum31=ysum31//counter

    xsum32=0
    ysum32=0
    counter=0
    for y in range(211,239):
        y=y+1
        for x in range(486,520):
    	    px=cv_image[y,x]
            if px>=200:
                xsum32=xsum32+x
                ysum32=ysum32+y
                counter=counter+1
            x=x+1
    xsum32=xsum32//counter
    ysum32=ysum32//counter

    
    
    rospy.loginfo("----------------------------------------------------")
    rospy.loginfo(str(xsum11)+","+str(ysum11)+"	"+str(xsum12)+","+str(ysum12))
    rospy.loginfo(str(xsum21)+","+str(ysum21)+"	"+str(xsum22)+","+str(ysum22))
    rospy.loginfo(str(xsum31)+","+str(ysum31)+"	"+str(xsum32)+","+str(ysum32))
    

   
    intrinsic=np.array([ [383.7944641113281, 0.0              , 322.3056945800781 ],
                         [0.0              , 383.7944641113281, 241.67051696777344],
                         [0.0              , 0.0              , 1.0               ] ])
    distortion=np.array([0, 0, 0, 0, 0])
    imagep=np.array([ [xsum31, xsum32, xsum21, xsum22, xsum11, xsum12],
                      [ysum31, ysum32, ysum21, ysum22, ysum11, ysum12] ])
    worldp=np.array([ [0.5, 0.5, 0.8, 0.8, 1.1, 1.1 ],
                      [0.2,-0.2, 0.2,-0.2, 0.2,-0.2 ], 
                      [0  , 0  , 0  , 0  , 0  , 0   ] ], dtype=np.float32)
    #retval, rvec, tvec= cv2.solvePnP(worldp,imagep,intrinsic,distortion)
    
    rospy.loginfo(" ")    
    rospy.loginfo("solvePnP results: ")    
    #rospy.loginfo("retval= "+str(retval)+",	rvec= "+str(rvec)+",	tvec= "+str(tvec))    
        
        
        
    
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
