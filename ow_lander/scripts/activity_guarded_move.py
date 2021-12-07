#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import math
import constants
import rospy
from utils import is_shou_yaw_goal_in_range


def arg_parsing(req):
  """
  :type req: class 'ow_lander.srv._GuardedMove.GuardedMoveRequest'
  """
  if req.use_defaults:
    # Default trenching values
    target_x = 2.0
    target_y = 0.0
    target_z = 0.3
    direction_x = 0.0
    direction_y = 0.0
    direction_z = 1.0
    search_distance = 0.5
  else:
    target_x = req.x
    target_y = req.y
    target_z = req.z
    direction_x = req.direction_x
    direction_y = req.direction_y
    direction_z = req.direction_z
    search_distance = req.search_distance

  return [req.use_defaults, target_x, target_y, target_z,
          direction_x, direction_y, direction_z, search_distance]


def pre_guarded_move(move_arm, args):
  """
  :type move_arm: class 'moveit_commander.move_group.MoveGroupCommander'
  :type args: List[bool, float, float, float, float, float, float, float]
  """
  targ_x = args[1]
  targ_y = args[2]
  targ_z = args[3]
  direction_x = args[4]
  direction_y = args[5]
  direction_z = args[6]
  search_distance = args[7]

  # STUB: GROUND HEIGHT TO BE EXTRACTED FROM DEM
  targ_elevation = -0.2
  if (targ_z+targ_elevation) == 0:
    offset = search_distance
  else:
    offset = (targ_z*search_distance)/(targ_z+targ_elevation)

  # Compute shoulder yaw angle to target
  alpha = math.atan2((targ_y+direction_y*offset)-constants.Y_SHOU,
                     (targ_x+direction_x*offset)-constants.X_SHOU)
  h = math.sqrt(pow((targ_y+direction_y*offset)-constants.Y_SHOU, 2) +
                pow((targ_x+direction_x*offset)-constants.X_SHOU, 2))
  l = constants.Y_SHOU - constants.HAND_Y_OFFSET
  beta = math.asin(l/h)

  # Move to pre move position, align shoulder yaw
  joint_goal = move_arm.get_current_joint_values()
  joint_goal[constants.J_DIST_PITCH] = 0
  joint_goal[constants.J_HAND_YAW] = 0
  joint_goal[constants.J_PROX_PITCH] = -math.pi/2
  joint_goal[constants.J_SHOU_PITCH] = math.pi/2
  joint_goal[constants.J_SHOU_YAW] = alpha + beta

  # If out of joint range, abort
  if (is_shou_yaw_goal_in_range(joint_goal) == False):
    return False

  joint_goal[constants.J_SCOOP_YAW] = 0
  move_arm.go(joint_goal, wait=True)
  move_arm.stop()

  # Once aligned to move goal and offset, place scoop tip at surface target offset
  goal_pose = move_arm.get_current_pose().pose
  goal_pose.position.x = targ_x
  goal_pose.position.y = targ_y
  goal_pose.position.z = targ_z
  move_arm.set_pose_target(goal_pose)
  _, plan, _, _ = move_arm.plan()
  if len(plan.joint_trajectory.points) == 0:  # If no plan found, abort
    return False

  plan = move_arm.go(wait=True)
  move_arm.stop()
  move_arm.clear_pose_targets()

  rospy.loginfo("Done planning approach of guarded_move")

  return True


def guarded_move_plan(move_arm, args):
  """
  :type move_arm: class 'moveit_commander.move_group.MoveGroupCommander'
  :type args: List[bool, float, float, float, float, float, float, float]
  """
  direction_x = args[4]
  direction_y = args[5]
  direction_z = args[6]
  search_distance = args[7]

  # Drive scoop tip along norm vector, distance is search_distance
  goal_pose = move_arm.get_current_pose().pose
  goal_pose.position.x -= direction_x*search_distance
  goal_pose.position.y -= direction_y*search_distance
  goal_pose.position.z -= direction_z*search_distance
  waypoints = [goal_pose]
  plan, _ = move_arm.compute_cartesian_path(waypoints, 0.01, 0.0)
  rospy.loginfo("Done planning safe part of guarded_move")
  return plan
