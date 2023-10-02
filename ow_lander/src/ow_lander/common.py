# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

"""Defines functions required by multiple modules within the package"""

import rospy

from math import pi, tau
from std_msgs.msg import Header

class Singleton(type):
  """When passed to the metaclass parameter in the class definition, the class
  will behave like a singleton.
  """
  _instances = {}
  def __call__(cls, *args, **kwargs):
    if cls not in cls._instances:
      cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
    return cls._instances[cls]

def normalize_radians(angle):
  """Returns a version of the angle between [-pi, pi)
  angle - in radians
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

def create_header(frame_id, timestamp=rospy.Time(0)):
  return Header(0, timestamp, frame_id)

def wait_for_subscribers(publisher, timeout, at_least=1, frequency=10):
  """Wait for subscribers to a topic.
  publisher - ROS publisher topic
  timeout   - Maximum time in seconds the function will block for
  at_least  - At least this many subscribers must be seen before the function
              will return (default: 1)
  frequency - Rate in Hz at which subscriber count will be checked
              (default: 10 Hz)
  """
  r = rospy.Rate(frequency)
  for _i in range(int(timeout * frequency)):
    if publisher.get_num_connections() >= at_least:
      return True
    r.sleep()
  return False
