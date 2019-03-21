#!/usr/bin/env python

# __BEGIN_LICENSE__
# Copyright (c) 2018-2019, United States Government as represented by the
# Administrator of the National Aeronautics and Space Administration. All
# rights reserved.
# __END_LICENSE__

import sys
import rosbag
import subprocess, os, signal
import copy
import rospy
import time
import csv
import datetime
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from moveit_commander.conversions import pose_to_list

## GLOBAL VARS ##
J_SCOOP_YAW = 5
J_HAND_YAW = 4
J_DIST_PITCH = 3 
J_PROX_PITCH = 2
J_SHOU_PITCH = 1
J_SHOU_YAW = 0

X_SHOU = 0.79
Y_SHOU = 0.175
HAND_Y_OFFSET = 0.0249979319838 
GROUND_POSITION = -0.175
SCOOP_OFFSET = 0.215

X_DELIV = 0.2
Y_DELIV = 0.2
Z_DELIV = 1.2
SHOU_YAW_DELIV = 0.4439

class MoveGroupPythonInteface(object):
  def __init__(self):
    super(MoveGroupPythonInteface, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('path_planning_commander', anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "arm"
    group_limbs = "limbs"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    move_limbs = moveit_commander.MoveGroupCommander(group_limbs)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    planning_frame = move_group.get_planning_frame()
    eef_link = move_group.get_end_effector_link()
    group_names = robot.get_group_names()

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.move_limbs = move_limbs
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def go_home(self):
    # Move to home position
    move_group = self.move_group
    joint_goal = move_group.get_current_joint_values()
    joint_goal[J_DIST_PITCH] = 3.1416
    joint_goal[J_HAND_YAW] = 0
    joint_goal[J_PROX_PITCH] = -2.75
    joint_goal[J_SHOU_PITCH] = 1.5708
    joint_goal[J_SHOU_YAW] = -1.5
    joint_goal[J_SCOOP_YAW] = 0
    move_group.go(joint_goal, wait=True)
    move_group.stop()

  def dig_trench(self, x_tr, y_tr, depth):
    move_group = self.move_group
    move_limbs = self.move_limbs

    # Add tests on depth and x y reach

    # Compute shoulder yaw angle to trench
    alpha = math.atan2(y_tr-Y_SHOU, x_tr-X_SHOU)
    h = math.sqrt( pow(y_tr-Y_SHOU,2) + pow(x_tr-X_SHOU,2) )
    l = Y_SHOU - HAND_Y_OFFSET
    beta = math.asin (l/h)

    # Move to pre trench position, align shoulder yaw
    joint_goal = move_group.get_current_joint_values()
    joint_goal[J_DIST_PITCH] = 0
    joint_goal[J_HAND_YAW] = math.pi/2.2
    joint_goal[J_PROX_PITCH] = -math.pi/2
    joint_goal[J_SHOU_PITCH] = math.pi/2
    joint_goal[J_SHOU_YAW] = alpha + beta
    joint_goal[J_SCOOP_YAW] = 0
    move_group.go(joint_goal, wait=True)
    move_group.stop()

    # Once aligned to trench goal, place hand above trench middle point
    goal_pose = move_limbs.get_current_pose().pose
    goal_pose.position.x = x_tr
    goal_pose.position.y = y_tr
    goal_pose.position.z = GROUND_POSITION + SCOOP_OFFSET - depth
    move_limbs.set_pose_target(goal_pose)
    plan = move_limbs.go(wait=True)
    move_limbs.stop()
    move_limbs.clear_pose_targets()

    # Rotate hand yaw to dig in
    joint_goal = move_group.get_current_joint_values()
    joint_goal[J_HAND_YAW] = 0
    move_group.go(joint_goal, wait=True)
    move_group.stop()

    # Insert here for linear trenching

    # Rotate hand yaw to dig out
    joint_goal = move_group.get_current_joint_values()
    joint_goal[J_HAND_YAW] = -math.pi/2.2
    move_group.go(joint_goal, wait=True)
    move_group.stop()

    # Go back to safe position and align yaw to deliver
    joint_goal = move_group.get_current_joint_values()
    joint_goal[J_DIST_PITCH] = 0
    joint_goal[J_HAND_YAW] = -math.pi/2
    joint_goal[J_PROX_PITCH] = -math.pi/2
    joint_goal[J_SHOU_PITCH] = math.pi/2
    joint_goal[J_SHOU_YAW] = SHOU_YAW_DELIV
    joint_goal[J_SCOOP_YAW]= 0
    move_group.go(joint_goal, wait=True)
    move_group.stop()

    # Go to deliver position
    joint_goal = move_group.get_current_joint_values()
    joint_goal[J_PROX_PITCH]= math.pi/2 - 0.1
    joint_goal[J_SCOOP_YAW]= math.pi - 0.05
    move_group.go(joint_goal, wait=True)
    move_group.stop()

    # Deliver (high amplitude)
    joint_goal = move_group.get_current_joint_values()
    joint_goal[J_HAND_YAW] = -math.pi
    move_group.go(joint_goal, wait=True)
    move_group.stop()
    joint_goal[J_HAND_YAW] = math.pi/2
    move_group.go(joint_goal, wait=True)
    move_group.stop()

def csv_to_yamls(filename):
  rows = [] 
  out = []
  nb_links = 6

  for x in range(nb_links):
    out.append(open(filename[:-4] + "_j" + str(x+1) + ".yaml","w"))

  # reading csv file 
  with open(filename, 'r') as csvfile: 
    # creating a csv reader object 
    csvreader = csv.reader(csvfile) 
  
    # extracting each data row one by one 
    for row in csvreader: 
      rows.append(row) 
  
  for row in rows[1:]: 
    for x in range(nb_links):
      out[x].write("---\n")
      out[x].write("%14s\n"%row[12+x])

  for x in range(nb_links):
    out[x].write("...")
    out[x].close()

def main():
  try:
    interface = MoveGroupPythonInteface()
    trench_x = rospy.get_param('/path_planning_commander/trench_x')
    trench_y = rospy.get_param('/path_planning_commander/trench_y')
    trench_d = rospy.get_param('/path_planning_commander/trench_d')

    # Home robot
    interface.go_home()
    
    # Start rosbag recording
    currentDT = datetime.datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
    location = "trajectory_"
    bagname = location + currentDT
    command = "rosbag record -O " + bagname + " /joint_states"    
    p = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd='.')
    interface.dig_trench(trench_x,trench_y,trench_d)
    time.sleep(1)

    # Stop rosbag recording (TODO: clean process)
    os.system("killall -s SIGINT record")  
    time.sleep(1)

    # rosbag to csv
    trajname = bagname + ".csv" 
    command = "rostopic echo -p -b " + bagname + ".bag /joint_states > " + trajname
    print command
    os.system(command)
    time.sleep(1)

    # csv to yaml
    csv_to_yamls(trajname) 

    # Cleanup bag and csv
    command = "rm " + bagname + ".bag"
    os.system(command)
    command = "rm " + trajname
    os.system(command)

    # Kill rosmaster and exit
    time.sleep(1)
    os.system("ps -ef | grep rosmaster | grep -v grep | awk '{print $2}' | xargs kill")

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()



