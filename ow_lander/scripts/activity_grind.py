#!/usr/bin/env python2

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import constants
import math
import copy
from utils import is_shou_yaw_goal_in_range
from activity_full_digging_traj import go_to_Z_coordinate, change_joint_value

def arg_parsing(req):
  if req.use_defaults :
    # Default trenching values
    x_start = 1.5
    y_start = 0
    depth = 0.01
    length = 0.3
    radial = False
    ground_position = constants.DEFAULT_GROUND_HEIGHT
    delete_prev_traj = False

  else :
    x_start = req.x
    y_start = req.y
    depth = req.depth
    length = req.length
    radial = req.radial
    ground_position = req.ground_position
    delete_prev_traj = req.delete_prev_traj

  return [req.use_defaults, x_start, y_start, depth, length, radial, ground_position, delete_prev_traj]


def plan_cartesian_path(move_arm, length, alpha, radial):

  if radial==False:
    alpha = alpha - math.pi/2

  waypoints = []
  wpose = move_arm.get_current_pose().pose

  wpose.position.x += length*math.cos(alpha)
  wpose.position.y += length*math.sin(alpha)

  waypoints.append(copy.deepcopy(wpose))

  (plan, fraction) = move_arm.compute_cartesian_path(
                               waypoints,   # waypoints to follow
                               0.01,        # end effector follow step (meters)
                               0.0)         # jump threshold
  return plan, fraction

def grind(move_arm, move_limbs, args):

  move_arm.set_planner_id('RRTstar')

  x_start = 0.9*args[1]
  y_start = 0.9*args[2] 
  depth = args[3]
  length = args[4]
  radial = args[5]
  ground_position = args[6]

  # Compute shoulder yaw angle to trench
  alpha = math.atan2(y_start-constants.Y_SHOU, x_start-constants.X_SHOU)
  h = math.sqrt( pow(y_start-constants.Y_SHOU,2) + pow(x_start-constants.X_SHOU,2) )
  l = constants.Y_SHOU - constants.HAND_Y_OFFSET
  beta = math.asin (l/h)
  # Move to pre trench position, align shoulder yaw
  joint_goal = move_arm.get_current_joint_values()
  joint_goal[constants.J_DIST_PITCH] = math.pi/6
  joint_goal[constants.J_HAND_YAW] = 4*math.pi/3
  joint_goal[constants.J_PROX_PITCH] = -math.pi/2
  joint_goal[constants.J_SHOU_PITCH] = math.pi/3
  joint_goal[constants.J_SHOU_YAW] = alpha + beta
  alpha = alpha+beta-0.09


  # If out of joint range, abort
  if (is_shou_yaw_goal_in_range(joint_goal) == False):
    return False

  move_arm.go(joint_goal, wait=True)
  move_arm.stop()

  # entering terrain
  z_start = ground_position + constants.GRINDER_HEIGHT - depth
  go_to_Z_coordinate(move_limbs, x_start, y_start, z_start)

  # grinding ice forward
  cartesian_plan, fraction = plan_cartesian_path(move_arm, length, alpha, radial)
  move_arm.execute(cartesian_plan, wait=True)
  move_arm.stop()

  joint_goal = move_arm.get_current_joint_values()
  change_joint_value(move_arm, constants.J_SHOU_YAW, joint_goal[0]+0.08)

  # grinding ice backwards
  cartesian_plan, fraction = plan_cartesian_path(move_arm, -length, alpha, radial)
  move_arm.execute(cartesian_plan, wait=True)
  move_arm.stop()


  # exiting terrain
  joint_goal = move_arm.get_current_joint_values()
  change_joint_value(move_arm, constants.J_SHOU_PITCH, joint_goal[1]+0.6)

  move_arm.set_planner_id('RRTconnect')

  return True
