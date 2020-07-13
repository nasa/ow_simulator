#!/usr/bin/env python

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import constants
import math
import copy
from tf.transformations import quaternion_from_euler
from utils import is_shou_yaw_goal_in_range
from activity_full_digging_traj import move_to_pre_trench_configuration
from activity_full_digging_traj import go_to_Z_coordinate, change_joint_value

def plan_cartesian_path_lin(move_arm, length):

  waypoints = []
  wpose = move_arm.get_current_pose().pose
  wpose.position.x += length
  waypoints.append(copy.deepcopy(wpose))

  (plan, fraction) = move_arm.compute_cartesian_path(
                               waypoints,   # waypoints to follow
                               0.01,        # end effector follow step (meters)
                               0.0)         # jump threshold
  return plan, fraction

def grind(move_arm, move_limbs, x_tr, y_tr, depth, length):

  pre_move_complete = move_to_pre_trench_configuration(move_arm, x_tr, y_tr)
  if pre_move_complete == False:
    return False

  # rotate hand
  change_joint_value(move_arm,constants.J_HAND_YAW, -2*math.pi/3)

  # approaching and entering terrain - along -Z
  z_tr = constants.GROUND_POSITION + 3*constants.SCOOP_HEIGHT - depth
  go_to_Z_coordinate(move_limbs,x_tr,y_tr,z_tr)

  # grinding ice - along +X
  cartesian_plan, fraction = plan_cartesian_path_lin(move_arm, length)
  move_limbs.execute(cartesian_plan, wait=True)
  move_limbs.stop()

  # exiting terrain - along +Z
  z_tr = constants.GROUND_POSITION + constants.GRIND_OFFSET - depth
  go_to_Z_coordinate(move_arm,x_tr,y_tr,z_tr)


  return True
