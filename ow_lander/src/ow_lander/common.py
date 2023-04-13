# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

"""Defines functions required by multiple modules within the package"""

from math import pi, tau
from ow_lander import constants
from std_msgs.msg import Header
from rospy import Time

from urdf_parser_py.urdf import URDF


class Singleton(type):
  """When passed to the metaclass parameter in the class definition, the class
  will behave like a singleton.
  """
  _instances = {}
  def __call__(cls, *args, **kwargs):
    if cls not in cls._instances:
      cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
    return cls._instances[cls]

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
  :return: (float) the angle in [-pi, pi)
  """
  # Note: tau = 2 * pi
  return (angle + pi) % tau - pi

def radians_equivalent(angle1, angle2, tolerance) :
  return abs(normalize_radians(angle1 - angle2)) <= tolerance

def in_closed_range(val, lo, hi):
  """
  :param val, lo, hi: (any mutually comparable types)
  :return: (bool)
  """
  return val >= lo and val <= hi

def create_header(frame_id, timestamp=Time(0)):
  return Header(0, timestamp, frame_id)
