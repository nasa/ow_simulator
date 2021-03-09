#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import constants
import roslib; roslib.load_manifest('urdfdom_py')
from urdf_parser_py.urdf import URDF

def is_shou_yaw_goal_in_range(joint_goal):      
  """
  # type joint_goal: List[float, float, float, float, float, float]
  """
  # If shoulder yaw goal angle is out of joint range, abort
  upper = URDF.from_parameter_server().joint_map["j_shou_yaw"].limit.upper
  lower = URDF.from_parameter_server().joint_map["j_shou_yaw"].limit.lower

  if (joint_goal[constants.J_SHOU_YAW]<lower) or (joint_goal[constants.J_SHOU_YAW]>upper):
    return False
    
  else:
    return True
