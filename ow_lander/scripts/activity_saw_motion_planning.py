#!/usr/bin/env python

# __BEGIN_LICENSE__
# Copyright (c) 2018-2019, United States Government as represented by the
# Administrator of the National Aeronautics and Space Administration. All
# rights reserved.
# __END_LICENSE__

import constants
import math
import copy
from tf.transformations import quaternion_from_euler
from utils import is_shou_yaw_goal_in_range
from activity_full_digging_traj import plan_cartesian_path
from activity_full_digging_traj import move_to_pre_trench_configuration


def saw_motion_planning(move_arm, move_limbs, x_tr, y_tr, depth):

  pre_move_complete = move_to_pre_trench_configuration(move_arm, x_tr, y_tr)
  if pre_move_complete == False:
    return False

  # rotate hand
  joint_goal = move_arm.get_current_joint_values()
  joint_goal[constants.J_HAND_YAW] = -2*math.pi/3
  move_arm.go(joint_goal, wait=True)
  move_arm.stop()

  # approaching and entering terrain - along -Z
  goal_pose = move_arm.get_current_pose().pose
  goal_pose.position.z = constants.GROUND_POSITION - depth

  move_arm.set_pose_target(goal_pose)
  plan = move_arm.plan()

  if len(plan.joint_trajectory.points) == 0: # If no plan found, abort
     return False

  plan = move_arm.go(wait=True)
  move_arm.stop()
  move_arm.clear_pose_targets()

  # sawing ice - along +X
  cartesian_plan, fraction = plan_cartesian_path(move_arm,move_limbs, scale=100)
  move_limbs.execute(cartesian_plan, wait=True)
  move_limbs.stop()

  # exiting terrain - along +Z
  goal_pose = move_arm.get_current_pose().pose
  goal_pose.position.z = constants.GROUND_POSITION + constants.SAW_OFFSET - depth

  move_arm.set_pose_target(goal_pose)
  plan = move_arm.plan()

  if len(plan.joint_trajectory.points) == 0: # If no plan found, abort
    return False

  plan = move_arm.go(wait=True)
  move_arm.stop()
  move_arm.clear_pose_targets()

  return True
