#!/usr/bin/env python

import roslib
import rospy
import math
import tf
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap
from Queue import PriorityQueue
import AStar_find_path
import Marker
import lineMarker
import motor_control


class robo_operation:
  def __init__(self):
    rospy.init_node("robo_operation")
    rospy.wait_for_message("/base_pose_ground_truth",Odometry)
    self.currentRealPose = self.get_init_pose()
    self.prevRealPose = self.get_init_pose()
    rospy.Subscriber("/base_pose_ground_truth",Odometry,self.set_real_pose)
    self.Goals = self.get_goals()
    self.grid = self.get_map()
    goalsQueue = PriorityQueue()
    for g in self.Goals:
      if type(g[0])==type(0.0) or type(g[1])==type(0.0):
        goalsQueue.put((0,(round(g[0]),round(g[1]))))
      else:
        goalsQueue.put((0,(g[0],g[1])))

    self.priorityGoals = AStar_find_path.goals_order((self.currentRealPose[0],self.currentRealPose[1]), goalsQueue)
    rospy.loginfo("Goals Sorted based on travel cost")
    rospy.loginfo(self.priorityGoals)
    self.pathMarker = lineMarker.lineMarkers("Path") #setup different markers
    self.drivenMarker = lineMarker.lineMarkers("Driven") 
    self.goalMarker = lineMarker.lineMarkers("Goals")
    self.completedMarker = lineMarker.lineMarkers("Completed")
    next_start = (self.currentRealPose[0],self.currentRealPose[1])
    current_Goal = []
    for goal in self.Goals: #goal marker
      self.goalMarker.lineMarker(goal,1.0,0,0,"/map") 
    for goal in self.priorityGoals:
      rospy.loginfo("Searching for a Path")
      current_path = AStar_find_path.Astar(next_start,goal,self.grid)
      if current_path:
        for pose in current_path:
          rospy.loginfo("Path Created")
          self.pathMarker.linemarker(pose,0,0,1.0,"/map")
        goal_index = 0
        for i in range(len(self.Goals)):
          float_goal = self.Goals[i]
          if round(fGoal[0]) == goal[0] and round(float_goal[1])==goal[1]:
            goal_index = i
            break
        next_start = (current_path[-1][0],current_path[-1][1])
        self.pathMarker.draw() #tags the desired paths on Rviz
        self.goalMarker.draw() #tags destinations on Rviz
    rospy.spin()
      

### QOF functions
  def get_map(self):
    rospy.wait_for_service('static_map')
    try:
      map_store = rospy.ServiceProxy('static_map',GetMap)
      return map_store().map
    except rospy.ServiceException:
      print "Failed to fetch map"

  def get_init_pose(self):
    start_pose = rospy.get_param("/robot_start")
    return start_pose

  def get_goals(self):
    goals = []
    goal = []
    for i in range(0,4):
      goalParam = "/goal%s" %str(i)
      goal = rospy.get_param(goalParam,[0.0,0.0])
      goals.append(goal)
    print goals
    return goals

  def set_real_pose(self,data):
    self.currentRealPose = [data.pose.pose.position.x,data.pose.pose.position.y]

try:
  robo_operation = robo_operation()
except KeyboardInterrupt:
  print "Interrupted"
  pass
