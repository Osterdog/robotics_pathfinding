#!/usr/bin/env python

import roslib
import rospy
import math
import tf
import tf2_ros
import Marker
from nav_msgs.msg import Odometry

class real_robot_pose:
  def __init__(self):
    rospy.init_node('real_robot_pose') #initializing this node
    rospy.Subscriber('/base_pose_ground_truth',Odometry,self.tfrm) #subscribing node contents to bpgt topic
    self.mkr = Marker.Markers()

  def tfrm(self,msg):
    br = tf.TransformBroadcaster()
    self.x = msg.pose.pose.position.x
    self.y = msg.pose.pose.position.y
    self.w = msg.pose.pose.orientation.w
    self.z = msg.pose.pose.orientation.z

    br.sendTransform((0,0,0),tf.transformations.quaternion_from_euler(0,0,0),rospy.Time.now(),"real_robot_pose","map") #transforming map to real_robot which subscribes to bpgt
    print  'Position: X = ',self.x,'Y = ', self.y,'|| Orientation: W = ', self.w, 'Z = ', self.z,'\n' #uses information from self transform (tfrm) to display bot location and orientation

#creating and drawing the markers
    self.mkr.add(self.x,self.y,self.w,self.z,0,1.0,0,"real_robot_pose")
    self.mkr.draw()

a = real_robot_pose()
rospy.spin() #keeps node running until manually stopped   
