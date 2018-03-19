#!/usr/bin/env python

import roslib
import rospy
import math
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class lineMarkers:
  def __init__(self,name):
    self.mkrID = 0
    self.linemarkers = []
    self.name = name
    self.linepub = rospy.Publisher(self.name, MarkerArray, queue_size = 50)

  def lineMarker(self,pose,r,g,b,frame):
    mkr = Marker()
    mkr.header.frame_id = frame
    mkr.ns = self.name
    mkr.id = self.mkrID
    mkr.type = mkr.CUBE
    mkr.action = mkr.ADD
    mkr.pose.position.x = pose[0]
    mkr.pose.position.y = pose[1]
    mkr.pose.orientation = 1
    mkr.scale.x = 0.7
    mkr.scale.y = 0.7
    mkr.scale.z = 0.2
    mkr.color.r = r
    mkr.color.g = g
    mkr.color.b = b
    mkr.color.a = 1.0
    self.linemarkers.append(mkr)
    self.mkrID += 1
    

  def draw(self):
    lineArray = MarkerArray()
    for m in self.linemarker:
      lineArray.linemarkers.append(m)
    self.linepub.publish(lineArray)
    self.mkrID = 0

  def clear(self):
    markerArray = MarkerArray()
    for m in self.markers:
      m.action = m.DELETE
      markerArray.markers.append(m)
    self.pub.publish(markerArray)
