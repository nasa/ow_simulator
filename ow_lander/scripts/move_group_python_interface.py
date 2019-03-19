#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman
# Mods: Antoine Tardy
##

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
GROUND_POSITION = 0.2
SCOOP_OFFSET = 0.2

X_DELIV = 0.2
Y_DELIV = 0.2
Z_DELIV = 1.2
SHOU_YAW_DELIV = 0.4439

def all_close(goal, actual, tolerance):
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

class MoveGroupPythonInteface(object):
  def __init__(self):
    super(MoveGroupPythonInteface, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface', anonymous=True)
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

    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

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
    joint_goal[J_HAND_YAW] = math.pi/2
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
    joint_goal[J_HAND_YAW] = -math.pi/2
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
    joint_goal[J_PROX_PITCH]= math.pi/2
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

    # For testing:
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

def csv_to_yamls(filename):
  rows = [] 
  out1 = open(filename[:-4] + "_j1.yaml","w")
  out2 = open(filename[:-4] + "_j2.yaml","w") 
  out3 = open(filename[:-4] + "_j3.yaml","w")
  out4 = open(filename[:-4] + "_j4.yaml","w") 
  out5 = open(filename[:-4] + "_j5.yaml","w")
  out6 = open(filename[:-4] + "_j6.yaml","w") 
  # reading csv file 
  with open(filename, 'r') as csvfile: 
    # creating a csv reader object 
    csvreader = csv.reader(csvfile) 
  
    # extracting each data row one by one 
    for row in csvreader: 
      rows.append(row) 
  
  for row in rows[1:]: 
    out1.write("---\n")
    out1.write("%14s\n"%row[12])
    out2.write("---\n")
    out2.write("%14s\n"%row[13])
    out3.write("---\n")
    out3.write("%14s\n"%row[14])
    out4.write("---\n")
    out4.write("%14s\n"%row[15])
    out5.write("---\n")
    out5.write("%14s\n"%row[16])
    out6.write("---\n")
    out6.write("%14s\n"%row[17])

  out1.write("...")
  out2.write("...")
  out1.close()
  out2.close()
  out3.write("...")
  out4.write("...")
  out3.close()
  out4.close()
  out5.write("...")
  out5.close()
  out6.write("...")
  out6.close()


def main():
  try:
    
    # Wait for rviz
    time.sleep(8)

    interface = MoveGroupPythonInteface()
    trench_x = rospy.get_param('/move_group_python_interface/trench_x')
    trench_y = rospy.get_param('/move_group_python_interface/trench_y')
    trench_d = rospy.get_param('/move_group_python_interface/trench_d')
    #interface.dig_trench(float(sys.argv[1]),float(sys.argv[2]),float(sys.argv[3]))
    
    # Start rosbag recording
    currentDT = datetime.datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
    location = "trajectory_"
    bagname = location + currentDT
    command = "rosbag record -O " + bagname + " /joint_states"    
    p = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd='.')
    interface.dig_trench(trench_x,trench_y,trench_d)
    time.sleep(4)
    os.system("killall -s SIGINT record")  
    time.sleep(2)

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

    time.sleep(1)

    os.system("ps -ef | grep rosmaster | grep -v grep | awk '{print $2}' | xargs kill")

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()



