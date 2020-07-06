#!/usr/bin/env python

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import constants
import math
import copy
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from moveit_msgs.msg import RobotState, Constraints, OrientationConstraint, PositionConstraint
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from shape_msgs.msg import SolidPrimitive

def arg_parsing(req):
  if req.use_defaults :
    x_drop=1.5
    y_drop=0
    delete_prev_traj=False

  else :
    trench_x=req.x_drop
    trench_y=req.y_drop
    delete_prev_traj=req.delete_prev_traj

  return [req.use_defaults,x_drop,y_drop,delete_prev_traj]


def discard(move_arm, x_drop, y_drop):

  # move_arm.set_planner_id("RRTstar")

  # x_drop, y_drop are coordinates

  # the beginning is like sample delivery
  mypi = 3.14159
  d2r = mypi/180
  r2d = 180/mypi

  goal_pose = move_arm.get_current_pose().pose
  #position was found from rviz tool
  goal_pose.position.x = x_drop
  goal_pose.position.y = y_drop
  goal_pose.position.z = 0.2 # was .78

  r = -179
  p = -20
  y = -90

  q = quaternion_from_euler(r*d2r, p*d2r, y*d2r)
  goal_pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

  move_arm.set_pose_target(goal_pose)

  plan = move_arm.plan()

  if len(plan.joint_trajectory.points) == 0: # If no plan found, abort
    return False

  plan = move_arm.go(wait=True)
  move_arm.stop()

  ###rotate scoop to deliver sample at current location...

  # adding position constraint on the solution so that the tip doesnot diverge to get to the solution.
  pos_constraint = PositionConstraint()
  pos_constraint.header.frame_id = "base_link"
  pos_constraint.link_name = "l_scoop"
  pos_constraint.target_point_offset.x = 0.1
  pos_constraint.target_point_offset.y = 0.1
  pos_constraint.target_point_offset.z = 0.1  ###rotate scoop to deliver sample at current location begin
  pos_constraint.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01]))
  pos_constraint.weight = 1

  #using euler angles for own verification..

  r = +180
  p = 90  # 45 worked get
  y = -90
  q = quaternion_from_euler(r*d2r, p*d2r, y*d2r)
  goal_pose = move_arm.get_current_pose().pose
  rotation = (goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z, goal_pose.orientation.w)
  euler_angle = euler_from_quaternion(rotation)

  goal_pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
  move_arm.set_pose_target(goal_pose)
  plan = move_arm.plan()

  if len(plan.joint_trajectory.points) == 0: # If no plan found, abort
    return False

  plan = move_arm.go(wait=True)
  move_arm.stop()
  move_arm.clear_pose_targets()

  move_arm.set_planner_id("RRTconnect")

  return True
