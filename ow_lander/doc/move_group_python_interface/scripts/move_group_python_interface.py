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
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
from std_msgs.msg import String
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
    
def main():
  try:
 
    if len(sys.argv) != 4 :
      print "Input error. Usage: <x_trench> <y_trench> <depth_trench>"
      sys.exit()
    interface = MoveGroupPythonInteface()
    interface.dig_trench(float(sys.argv[1]),float(sys.argv[2]),float(sys.argv[3]))

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

