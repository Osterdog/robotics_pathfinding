#!/usr/bin/env python

import roslib
import rospy
import math
from sys import maxint
from Queue import PriorityQueue

#Selects closest goal based on heuristics
def goals_order(start_pose,goals,new_order_goals=[]):
  rem_pose = PriorityQueue()
  if goals.qsize() >1:
    for i in range(goals.qsize()):
      pose = goals.get()[1]
      first = cost_h(pose,start_pose)
      rem_pose.put((first,pose))
    near = rem_pose.get()
    new_order_goals.append(near[1])
    return goals_order(near[1],rem_pose,new_order_goals)
  else:
    last = goals.get()[1]
    new_order_goals.append(last)
    return new_order_goals

#determining cost to move to location
def cost_h(pose,dest):
  cost = abs(pose[0]-dest[0])+abs(pose[1]-dest[1])
  return cost

#A* pathfinding
def Astar(start,goal,grid):
  TravelQueue = PriorityQueue() #gets queue list
  TravelQueue.put((0,start))
  parents = {}
  cost = {}
  parents[start] = None
  cost[start] = 0
  path = []
  while not TravelQueue.empty():
    curr_pose = TravelQueue.get()[1]
    if curr_pose == goal:
      while curr_pose != start:
        path.insert(0,[curr_pose[0],curr_pose[1]])
        curr_pose = parents[curr_pose]
      return path

    for child in find_kids(curr_pose, parents[curr_pose],grid):
      cost_new = cost[curr_pose]+1
      if child not in cost or cost_new < cost[child]:
        cost[child] = cost_new
        priority = cost_new + cost_h(child,goal)
        parents[child] = curr_pose
        TravelQueue.put((priority,child))
  print "No path available"
  return False

#Checking nearby squares/cells
def find_kids(pose,parent_pose,grid):
  kids = []
  acc = 1
  #checking compass (N,S,E,W)
  if occupy([pose[0]-acc,pose[1]],grid) and (pose[0]-acc,pose[1]) != parent_pose:
    kids.append((pose[0]-acc,pose[1]))
  if occupy([pose[0]+acc,pose[1]],grid) and (pose[0]+acc,pose[1]) != parent_pose:
    kids.append((pose[0]+acc,pose[1]))
  if occupy([pose[0],pose[1]-acc],grid) and (pose[0],pose[1]-acc) != parent_pose:
    kids.append((pose[0],pose[1]-acc))
  if occupy([pose[0],pose[1]+acc],grid) and (pose[0],pose[1]+acc) != parent_pose:
    kids.append((pose[0],pose[1]+acc))
  return kids

def occupy(location,grid):
  if location:
    x = int(round((location[0]-grid.info.origin.position.x)/grid.info.resolution))
    y = int(round((location[1]-grid.info.origin.position.y)/grid.info.resolution))
    index = x+y*grid.info.width
    return free_cells(index,grid)
    #free space for movement. Cell size determines how large individual cells are. Value can be changed easily
def free_cells(index,grid):
  cell_size = 12
  row = 0
  index_start = (index-cell_size)-grid.info.width*cell_size
  line = math.ceil(index_start/grid.info.width-1)
  for i in range(2*cell_size):
    for j in range(2*cell_size):
      try:
        if(grid.data[index_start+j+row] !=0 or line != math.ceil((index_start+j)/grid.info.width-1)):
          return False
        elif(j==9):
          row = row + grid.info.width
      except IndexError:
        return False
  return True
