#!/usr/bin/env python
from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
from scipy import interpolate
import math
from nav_msgs.msg import Odometry
from autominy_msgs.msg import NormalizedSteeringCommand
from autominy_msgs.msg import SpeedCommand
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from scipy.spatial import distance
from operator import add 

def nothing(x):
    pass
'''
img = np.zeros((300,512,3), np.uint8)
cv2.namedWindow('image')
cv2.createTrackbar('kp','image',0,20,nothing)
cv2.createTrackbar('ki','image',0,100,nothing)
cv2.createTrackbar('kd','image',0,100,nothing)
'''

class lanes: 

  def spline(self):
  
    
    arcLengthArray1=self.lane1[:,0]
    xArray1=self.lane1[:,1]
    yArray1=self.lane1[:,2]
    arcLengthArray2=self.lane2[:,0]
    xArray2=self.lane2[:,1]
    yArray2=self.lane2[:,2]
    
	
    xCs1=interpolate.CubicSpline(arcLengthArray1, xArray1)
    yCs1=interpolate.CubicSpline(arcLengthArray1, yArray1)
    xCs2=interpolate.CubicSpline(arcLengthArray2, xArray2)
    yCs2=interpolate.CubicSpline(arcLengthArray2, yArray2)
 
    x1coord=xCs1(self.lane1Orig[:,0])
    y1coord=yCs1(self.lane1Orig[:,0])
    x2coord=xCs2(self.lane2Orig[:,0])
    y2coord=yCs2(self.lane2Orig[:,0])

    self.markerPoints1=[Point(x,y,0) for x,y in zip(x1coord,y1coord) ]
    
    
    self.markerPoints2=[Point(x,y,0) for x,y in zip(x2coord,y2coord) ]

    markerLane1=self.makeMarkerLineStrip(self.markerPoints1, 0)
    markerLane2=self.makeMarkerLineStrip(self.markerPoints2, 1)

    #subscriber
    rate = rospy.Rate(10) # 10hz
    self.clickPointSub = rospy.Subscriber("/clicked_point",PointStamped,self.callbackClosest)

    while not rospy.is_shutdown():
        self.pub.publish(markerLane1)
        self.pub.publish(markerLane2)
        rate.sleep()
       # self.clickPointSub = rospy.Subscriber("/clicked_point",PointStamped,self.callbackClosest)

  def makeMarkerLineStrip(self, points, iD): 

      marker1 = Marker()
      marker1.header.frame_id="/map"
      marker1.id = iD
      marker1.action = marker1.ADD
      marker1.pose.position.x = 0
      marker1.pose.position.y = 0
      marker1.pose.position.z = 0
      marker1.pose.orientation.x = 0.0
      marker1.pose.orientation.y = 0.0
      marker1.pose.orientation.z = 0.0
      marker1.pose.orientation.w = 1.0 
      marker1.scale.x = 0.1
      marker1.scale.y = 0.1
      marker1.scale.z = 0.1
      marker1.color.a = 1.0
      marker1.color.r = 0.0
      marker1.color.g = 1.0
      marker1.color.b = 0.0
      marker1.type=marker1.LINE_STRIP
      marker1.points=points

      return marker1

  def makeMarkerSphere(self, point, iD): 

      marker1 = Marker()
      marker1.header.frame_id="/map"
      marker1.id = iD
      marker1.action = marker1.ADD
      marker1.pose.position.x = point.x
      marker1.pose.position.y = point.y
      marker1.pose.position.z = 0
      marker1.pose.orientation.x = 0.0
      marker1.pose.orientation.y = 0.0
      marker1.pose.orientation.z = 0.0
      marker1.pose.orientation.w = 1.0 
      marker1.scale.x = 0.5
      marker1.scale.y = 0.5
      marker1.scale.z = 0.5
      marker1.color.a = 1.0
      marker1.color.r = 0.0
      marker1.color.g = 0.0
      marker1.color.b = 1.0
      marker1.type=marker1.SPHERE
      

      return marker1
  def la_help(self, clickedPoint, closMarkerPoint, markerPoints):
    #lookahead point

    lookaheadDist=1 #could be anything

    #get orthogonal unit vector to vector from closest point to clicked point 
    vectorClickedClosest=[(clickedPoint.x-closMarkerPoint.x), (clickedPoint.y-closMarkerPoint.y),0]
    vLength=np.linalg.norm(vectorClickedClosest)
    orthUnitVector=[((-vectorClickedClosest[0])/vLength), vectorClickedClosest[1]/vLength]
    
    #now point on this vector with lookahead distance
    pointOrthVec= list(map(add, [clickedPoint.x, clickedPoint.y] ,(map(lambda x: x*lookaheadDist, orthUnitVector))))
    
    #get the closest point on lanes to pointOrthVec
    pOrthVecP=Point(pointOrthVec[0], pointOrthVec[1] ,0)
    return( self.binSearchCP( pOrthVecP , markerPoints ) )
    

  def closestAndLookaheadPoint(self, clickedPoint):
    #closest point

    #check which lane has the closer point
    
    closPL1=self.binSearchCP(clickedPoint, self.markerPoints1)
    closPL2=self.binSearchCP(clickedPoint, self.markerPoints2)

    if (closPL1[1]<=closPL2[1]):
	closMarkerPoint=Point(closPL1[0][0],closPL1[0][1],0)
        laP1=self.la_help(clickedPoint, closMarkerPoint, self.markerPoints1)
	laMarkerPoint=Point(laP1[0][0],laP1[0][1],0)
    else:
        closMarkerPoint=Point(closPL2[0][0],closPL2[0][1],0)
        laP2=self.la_help(clickedPoint, closMarkerPoint, self.markerPoints2)
	laMarkerPoint=Point(laP2[0][0],laP2[0][1],0)

    '''
    #lookahead point

    lookaheadDist=3 #could be anything

    #get orthogonal unit vector to vector from closest point to clicked point 
    vectorClickedClosest=[(clickedPoint.x-closMarkerPoint.x), (clickedPoint.y-closMarkerPoint.y),0]
    vLength=np.linalg.norm(vectorClickedClosest)
    orthUnitVector=[((-vectorClickedClosest[0])/vLength), vectorClickedClosest[1]/vLength]
    
    #now point on this vector with lookahead distance
    pointOrthVec= list(map(add, [clickedPoint.x, clickedPoint.y] ,(map(lambda x: x*lookaheadDist, orthUnitVector))))
    
    #get the closest point on lanes to pointOrthVec
    pOrthVecP=Point(pointOrthVec[0], pointOrthVec[1] ,0)
    laP1=self.binSearchCP( pOrthVecP , self.markerPoints1)
    laP2=self.binSearchCP(pOrthVecP, self.markerPoints2)

    if (laP1[1]<=laP2[1]):
	laMarkerPoint=Point(laP1[0][0],laP1[0][1],0)
    else:
        laMarkerPoint=Point(laP2[0][0],laP2[0][1],0)
    '''
   

    #clicked point blue, closest point red, lookahead point yellow
    clickPMarker=self.makeMarkerSphere(clickedPoint, 4)
    
    closPMarker=self.makeMarkerSphere(closMarkerPoint, 3)
    closPMarker.color.a=1.0
    closPMarker.color.r=1.0
    closPMarker.color.g=0.0
    closPMarker.color.b=0.0

    laPMarker=self.makeMarkerSphere(laMarkerPoint, 5)
    laPMarker.color.a=1.0
    laPMarker.color.r=1.0
    laPMarker.color.g=1.0
    laPMarker.color.b=0.0

    self.pub.publish(clickPMarker)
    self.pub.publish(closPMarker)
    self.pub.publish(laPMarker)


    

  def binSearchCP(self, point, lane):
 

    
    pointAsList=[point.x,point.y]
    index=(len(lane)//2)

    
    currMinPoint=[lane[index].x, lane[index].y]
    dist=distance.euclidean(pointAsList, currMinPoint)
    currMinDist=dist
    

    #look for local minimum left or right from startPoint
  
    if (dist> distance.euclidean(pointAsList, [lane[index-1].x, lane[index-1].y])):

      while (index<len(lane) and index>0 and (dist> distance.euclidean(pointAsList, [lane[index-1].x, lane[index-1].y]))) :
       
         dist=distance.euclidean(pointAsList, [lane[index-1].x, lane[index-1].y])
         index-=1
      currMinDist=dist
      currMinPoint=[lane[index].x, lane[index].y]
      lane=lane[int(math.ceil(len(lane)/2)):]
    else:
      while ((index<(len(lane)-1)) and (index>0) and (dist> distance.euclidean(pointAsList, [lane[index+1].x, lane[index+1].y]))):

         dist=distance.euclidean(pointAsList, [lane[index+1].x, lane[index+1].y])
         index+=1
      currMinDist=dist
      currMinPoint=[lane[index].x,lane[index].y]
      lane=lane[:int(math.ceil(len(lane)/2))]

    #now do binary search on remaining half (the one without the local minimum)


    
    left=0
    right=len(lane)-1
    middle=left+(right-left)//2
    currPoint=[lane[middle].x,lane[middle].y]
    dist=distance.euclidean(pointAsList, currPoint)

    while((left<right) and ((dist>= distance.euclidean(pointAsList, [lane[middle-1].x, lane[middle-1].y])) or (dist>=distance.euclidean(pointAsList, [lane[middle+1].x, lane[middle+1].y] ))) ):
      
       if (dist>= distance.euclidean(pointAsList, [lane[middle-1].x, lane[middle-1].y])):
	  right=middle-1
          middle=left+((right-left)//2)
          currPoint=[lane[middle].x,lane[middle].y]
	  dist=distance.euclidean(pointAsList, currPoint)

       else:
	  left=middle+1
          middle=left+((right-left)//2)
          currPoint=[lane[middle].x,lane[middle].y]
	  dist=distance.euclidean(pointAsList, currPoint)         
	
      

    #check which point is closer
    #if (dist<currMinDist):
    return[currPoint, dist]

    #return [currMinPoint, currMinDist]

  
  def callbackClosest(self,data):
 
    self.closestAndLookaheadPoint(data.point)
	

    


  def __init__(self):
    self.lane1Orig=np.load("/home/sinanu/lane1.npy")
    self.markerPoints1=[]
    self.markerPoints2=[]
    
    self.lane2Orig=np.load("/home/sinanu/lane2.npy")
    # points from hint
    self.lane1 = self.lane1Orig[[0, 100, 150, 209, 259, 309, 350, 409, 509, 639, 750, 848, 948, 1028, 1148, 1276], :]
    self.lane2 = self.lane2Orig[[0, 50, 100, 150, 209, 400, 600, 738, 800, 850, 900, 949, 1150, 1300, 1476], :]
    self.pub = rospy.Publisher("/marker",Marker)
    


   
def main():
  rospy.init_node('lanes', anonymous=True)
  lane = lanes()
  lane.spline()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
