#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

from math import pi
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

def normalize_radians(angle):
  """
  :param angle: (float)
  :return: (float) the angle in [-pi, pi]
  """
  tolerance = 0.01
  while angle > (pi+tolerance):
    angle -= 2 * pi
  while angle < -(pi+tolerance):
    angle += 2 * pi
  return angle

def adjust_pan_radians(actual, goal):
  """
  :param actual, goal: (float)
  :return: (float)

  """
  tolerance = 0.01
  if actual > 0 and goal < 0):
    return actual - (2 * pi)
  if actual < 0 and goal > 0):
    return actual + (2 * pi)


def radians_equivalent (angle1, angle2, tolerance) :
  return (abs(angle1 - angle2) <= tolerance or
          abs((angle1 - 2 * pi) - angle2) <= tolerance or
          abs((angle1 + 2 * pi) - angle2) <= tolerance)

def in_range(val, lo, hi):
  """
  :param val, lo, hi: (any comparable type)
  :return: (bool)
  """
  return val >= lo and val <= hi
