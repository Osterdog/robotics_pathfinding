#!/usr/bin/env python

import roslib
import rospy
import math
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class Markers:
  def __init__(self):
    self.i = 0
    self.markers = []
    self.pub = rospy.Publisher("/real_robot_pose", MarkerArray, queue_size = 200)

  def add(self,x,y,w,z,r,g,b,frame):
    mr = Marker()
    mr.header.frame_id = frame
    mr.ns = "basic"
    mr.id = self.i
    mr.type = mr.ARROW
    mr.action = mr.ADD
    mr.pose.position.x = x
    mr.pose.position.y = y
    mr.pose.orientation.w = w
    mr.pose.orientation.z = z
    mr.scale.x = .6
    mr.scale.y = .05
    mr.scale.z = .05
    mr.color.r = r
    mr.color.g = g
    mr.color.b = b
    mr.color.a = 1.0
    self.markers.append(mr)
    self.i += 1
    

  def draw(self):
    markerArray = MarkerArray()
    for m in self.markers:
      markerArray.markers.append(m)
    self.pub.publish(markerArray)
    self.i = 0

  def clear(self):
    markerArray = MarkerArray()
    for m in self.markers:
      m.action = m.DELETE
      markerArray.markers.append(m)
    self.pub.publish(markerArray)
