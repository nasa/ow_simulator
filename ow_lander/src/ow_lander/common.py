# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

"""Defines functions required by multiple modules within the package"""

from math import pi, tau, isclose
from ow_lander import constants
# import roslib; roslib.load_manifest('urdfdom_py')
from urdf_parser_py.urdf import URDF
from tf.transformations import euler_from_quaternion

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

def _normalize_radians(angle):
  """
  :param angle: (float)
  :return: (float) the angle in [-pi, pi)
  """
  # Note: tau = 2 * pi
  return (angle + pi) % tau - pi

def radians_equivalent(angle1, angle2, tolerance) :
  return abs(_normalize_radians(angle1 - angle2)) <= tolerance

def in_closed_range(val, lo, hi, tolerance):
  """
  :param val, lo, hi, tolerance: (any mutually comparable types)
  :return: (bool)
  """
  return val >= lo-tolerance and val <= hi+tolerance

def poses_equivalent(pose1, pose2, \
    meter_tolerance=constants.ARM_POSE_METER_TOLERANCE, \
    radian_tolerance=constants.ARM_POSE_RADIAN_TOLERANCE):
  p1 = pose1.position
  p2 = pose2.position
  if isclose(p1.x, p2.x, abs_tol=meter_tolerance) \
      and isclose(p1.y, p2.y, abs_tol=meter_tolerance) \
      and isclose(p1.z, p2.z, abs_tol=meter_tolerance):
    o1 = pose1.orientation
    o2 = pose2.orientation
    # quaternions are degenerate, we must instead compare Euler angles
    r1, p1, y1 = euler_from_quaternion([o1.x, o1.y, o1.z, o1.w])
    r2, p2, y2 = euler_from_quaternion([o2.x, o2.y, o2.z, o2.w])
    if isclose(r1, r2, abs_tol=radian_tolerance) \
        and isclose(p1, p2, abs_tol=radian_tolerance) \
        and isclose(y1, y2, abs_tol=radian_tolerance):
      return True
  return False
