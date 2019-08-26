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
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
from scipy import interpolate
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

#diese bibliothek muss gedownloaded werden !!
from scipy.spatial import distance


def create_marker(i):
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.type = marker.LINE_STRIP
    marker.action = marker.ADD
    marker.id=i
    
    # marker scale
    marker.scale.x = 0.03
    marker.scale.y = 0.03
    marker.scale.z = 0.03

    # marker color
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0

    # marker orientaiton
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    
    # marker position
    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0
    
    return(marker)


class image_converter:

  def __init__(self):
    self.array1=[]
    self.array2=[]
    self.lane1orig=np.load("/home/sinanu/lane1.npy")
    self.lane2orig=np.load("/home/sinanu/lane2.npy")
    self.lane1 = self.lane1orig[[0, 100, 150, 209, 259, 309, 350, 409, 509, 639, 750, 848, 948, 1028, 1148, 1276], :]
    self.lane2 = self.lane2orig[[0, 50, 100, 150, 209, 400, 600, 738, 800, 850, 900, 949, 1150, 1300, 1476], :]
    self.marker_pub = rospy.Publisher("/marker",Marker)
    closest_sub = rospy.Subscriber("/clicked_point",PointStamped,self.callback_closest)
    lookahead_sub = rospy.Subscriber("/clicked_point",PointStamped,self.callback_lookahead)


  def spline(self):
    arcLenghtArray1=self.lane1[:,0]
    xArray1=self.lane1[:,1]
    yArray1=self.lane1[:,2]
    arcLenghtArray2=self.lane2[:,0]
    xArray2=self.lane2[:,1]
    yArray2=self.lane2[:,2]

    xCs1=interpolate.CubicSpline(arcLenghtArray1, xArray1)
    yCs1=interpolate.CubicSpline(arcLenghtArray1, yArray1)
    xCs2=interpolate.CubicSpline(arcLenghtArray2, xArray2)
    yCs2=interpolate.CubicSpline(arcLenghtArray2, yArray2)

    marker1=create_marker(1)
    x1coord=xCs1(self.lane1orig[:,0])
    y1coord=yCs1(self.lane1orig[:,0])  
    points1=[Point(x,y,0) for x,y in zip(x1coord,y1coord)]
    marker1.points=points1
   

    marker2=create_marker(2)  
    x2coord=xCs2(self.lane2orig[:,0])
    y2coord=yCs2(self.lane2orig[:,0])
    points2=[Point(x,y,0) for x,y in zip(x2coord,y2coord)]
    marker2.points=points2

    self.array1=points1
    self.array2=points2


    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
    
      self.marker_pub.publish(marker1)
      self.marker_pub.publish(marker2)
      rate.sleep()

  def callback_closest(self,data):
    print(data)
    cl=self.closest( (data.point.x, data.point.y) )
    m=create_marker(3)
    m.type=m.SPHERE
    m.color.r = 1.0
    m.color.g = 0.0
    m.color.b = 0.0
    m.scale.x = 0.2
    m.scale.y = 0.2
    m.scale.z = 0.2
    m.pose.position.x = data.point.x
    m.pose.position.y = data.point.y
    m.pose.position.z = 0.0
    m2=create_marker(4)
    m2.color.r = 0.0
    m2.color.g = 0.0
    m2.color.b = 1.0
    m2.scale.x = 0.2
    m2.scale.y = 0.2
    m2.scale.z = 0.2
    m2.type=m.SPHERE
    m2.pose.position.x = cl[0]
    m2.pose.position.y = cl[1]
    m2.pose.position.z = 0.0
    self.marker_pub.publish(m)
    self.marker_pub.publish(m2)

  def callback_lookahead(self,data):
    la_point=self.lookahead( (data.point.x, data.point.y) )
    m=create_marker(5)
    m.type=m.SPHERE
    m.color.r = 1.0
    m.color.g = 0.0
    m.color.b = 1.0
    m.scale.x = 0.2
    m.scale.y = 0.2
    m.scale.z = 0.2
    m.pose.position.x = data.point.x
    m.pose.position.y = data.point.y
    m.pose.position.z = 0.0
    m2=create_marker(6)
    m2.color.r = 0.0
    m2.color.g = 1.0
    m2.color.b = 1.0
    m2.scale.x = 0.2
    m2.scale.y = 0.2
    m2.scale.z = 0.2
    m2.type=m.SPHERE
    m2.pose.position.x = la_point[0]
    m2.pose.position.y = la_point[1]
    m2.pose.position.z = 0.0
    self.marker_pub.publish(m)
    self.marker_pub.publish(m2)


  
  def lookahead(self,point):
    lookahead_distance=0.5
    cl=self.closest(point)

    if cl[4]==1:
      dst=self.lane2orig[ cl[3] ][0]
      dst+=lookahead_distance
      if dst>self.lane1orig[-1][0]:
        dst-=self.lane1orig[-1][0]

      arcLenghtArray1=self.lane1[:,0]
      xArray1=self.lane1[:,1]
      yArray1=self.lane1[:,2]
      xCs1=interpolate.CubicSpline(arcLenghtArray1, xArray1)
      yCs1=interpolate.CubicSpline(arcLenghtArray1, yArray1)
      x=xCs1(dst)
      y=yCs1(dst)
    

    else:
      dst=self.lane2orig[ cl[3] ][0]
      dst+=lookahead_distance
      if dst>self.lane2orig[-1][0]:
        dst-=self.lane2orig[-1][0]

      arcLenghtArray2=self.lane2[:,0]
      xArray2=self.lane2[:,1]
      yArray2=self.lane2[:,2]
      xCs2=interpolate.CubicSpline(arcLenghtArray2, xArray2)
      yCs2=interpolate.CubicSpline(arcLenghtArray2, yArray2)
      x=xCs2(dst)
      y=yCs2(dst)
      
    lookahead_point=(x,y)
    return( lookahead_point )

  def closest(self,point):
    cl1=self.binarys(self.array1,len(self.array1),point)
    cl2=self.binarys(self.array2,len(self.array2),point)
    if cl1[2]<cl2[2]:
      return( (cl1[0],cl1[1],cl1[2], cl1[3], 1) )

    return( (cl2[0],cl2[1],cl2[2], cl2[3], 2) )
	
  def binarys(self,array,n,point):
    l=0
    r=n-1
    while True:
      m=(l+r)//2
      if m>=len(array)-1:
        #resetten und so tun als ob man am anfang nach rechts gegangen waere (wenn man rechts unten vom spline anfang klicken wuerde)
        l=0
        r=n-1
        m=(l+r)//2
        r=m-1
        m=(l+r)//2
        
      dst=distance.euclidean(point, (array[m].x,array[m].y) )

      if distance.euclidean(point, (array[m+1].x,array[m+1].y) ) < dst:
        l=m+1
      elif distance.euclidean(point, (array[m-1].x,array[m-1].y) ) < dst:
        r=m-1
      else:
        error=self.find_opposite_point_index(m,array)
        #ueberpruefen, ob der punkt auf der gegenueberliegenden seite naeher dran ist (wenn man in der mitte klickt)
        if distance.euclidean(point, (array[m].x,array[m].y) ) > distance.euclidean(point, (array[error].x,array[error].y) ):
          return( (array[error].x, array[error].y, dst, error) )
        return( (array[m].x, array[m].y, dst, m) )
        
  def find_opposite_point_index(self, m, array):
    dst=0
    if ( m> ( (len(array)-1) - (len(array)-1) //6) ) or ( m< ( (len(array)-1) //2 - (len(array)-1) //6) ):
      if  m> ( (len(array)-1) - (len(array)-1) //6) :
        dst= ( (len(array)-1) - (len(array)-1) //6) - m
      else:
        dst= -1*( (len(array)-1) //6 + m)
    else:
      dst= ( (len(array)-1) - (len(array)-1) //6) - m
    if ( ( (len(array)-1) - (len(array)-1) //6)+dst ) > (len(array)-1) :
      opposite= ( ( (len(array)-1) - (len(array)-1) //6)+dst ) - (len(array)-1) 
    else:
      opposite= ( (len(array)-1) - (len(array)-1) //6)+dst 
    return(opposite)



def main():
  rospy.init_node('a8-1', anonymous=True)
  ic = image_converter()
  ic.spline()

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
