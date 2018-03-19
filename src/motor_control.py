#!/usr/bin/env python

import roslib
import rospy
import math
import tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

#Controlling Robot Motion

class Motion:
  def __init__(self,start_pose):
    rospy.loginfo("Initialising")
    self.ctl_vel=rospy.Publisher('/cmd_vel',Twist,queue_size=10)
    self.cmd_vel=Twist()
    self.dimensions_xyz = self.bot_size()
    self.path = None
    self.goalPose = [0,0]
    self.GoalPose = [0,0]
    self.goalTheta = 0.0
    self.tf = tf.TransformListener()
    self.currentMapPose = start_pose
    self.lastOdomPose = [0,0,0]
    self.blocked = False
    self.goalReached = False
    rospy.wait_for_message("/odom", Odometry)
    rospy.Subscriber("/odom", Odometry, self.odom)

  def find_path(self,path,Goal):
    self.path = Path
    self.goalPose = self.path.pop(0)
    self.GoalPose = Goal

  def bot_size(self):
    x = 1
    y = 1
    z = 0.2
    rospy.set_param('/robot/dimensions_xyz',[x,y,z])
    return [x,y,z]

  def odom(self,data):
    location = data.pose.pose.position
    orientation = data.pose.pose.orientation
    self.currentMapPose[0] = self.currentMapPose[0]+(location.x - self.lastOdomPose[0])
    self.currentMapPose[1] = self.currentMapPose[1]+(location.y - self.lastOdomPose[1])
    Yaw = tf.transformations.euler_from_quaternion([0.0,0.0,orientation.z,orientation.w])[2]
    self.currentMapPose[2] = self.currentMapPose[2]+(Yaw - self.lastOdomPose[2])
    self.lastOdomPose[0] = location.x
    self.lastOdomPose[1] = location.y
    self.lastOdomPose[2] = Yaw
    self.goalTheta = math.atan2(self.goalPose[1]-self.currentMapPose[1],self.goalPose[0]-self.currentMapPose[0])

  def driving(self):
    if(not self.blocked):
      if len(self.path)==1 and (abs(self.GoalPose[0]-self.currentMapPose[0])>0.05 or abs(self.floatGoalMapPose[1]-self.currentMapPose[1])>0.05):
        self.goalTheta = math.atan2(self.goalPose[1]-self.currentMapPose[1],self.goalPose[0]-self.currentMapPose[0])
        self.goalPose = self.GoalPose
        self.cmd_vel.angular.z = self.adjust_rotation(self.goalTheta-self.currentMapPose[2])
        self.cmd_vel.linear.x = 0.2
      elif(abs(self.goalPose[0]-self.currentMapPose[0])>0.5 or abs(self.goalPose[1]-self.currentMapPose[1])>0.5):
        self.cmd_vel.angular.z = self.adjust_rotation(self.goalTheta-self.currentMapPose[2])
        if abs(self.cmd_vel.angular.z)>0.8:
          self.cmd_vel_linear.x = 0.0
        else:
          self.cmd_vel.linear.x = 1.2
      else:
        if len(self.path)==1:
          self.goalReached = True
        else:
          self.goalReached = False
        self.goalPose = self.path.pop(0)
        self.goalTheta = math.atan2(self.goalPose[1]-self.currentMapPose[1],self.goalPose[0]-self.currentMapPose[0])
        self.cmd_vel.angular.z = 0.0
        self.cmd_vel.linear.x = 0.0
    else:
      print "Path is blocked"
      self.cmd_vel.angular.z = 0.5
      self.cmd_vel.linear.x = 0.0
    self.ctl_vel.publish(self.cmd_vel)
  def rotation(self,theta):
    while theta <= math.pi:
      theta +=2*math.pi
    while theta > math.pi:
      theta -= 2*math.pi
    return theta




  
