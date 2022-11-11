#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

from math import pi, tau
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

def _normalize_radians(angle):
  """
  :param angle: (float)
  :return: (float) the angle in [-pi, pi)
  """
  # Note: tau = 2 * pi
  return (angle + pi) % tau - pi

def radians_equivalent (angle1, angle2, tolerance) :
  return abs(_normalize_radians(angle1 - angle2)) <= tolerance

def in_closed_range(val, lo, hi, tolerance):
  """
  :param val, lo, hi, tolerance: (any mutually comparable types)
  :return: (bool)
  """
  return val >= lo-tolerance and val <= hi+tolerance
