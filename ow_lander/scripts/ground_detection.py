#!/usr/bin/env python

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import time
import numpy as np
import rospy
import tf2_ros
from collections import deque
from gazebo_msgs.msg import LinkStates


GROUND_DETECTION_THRESHOLD = -0.005

class SlidingWindow:
  """
  A class that maintains a sliding window over a stream of samples in FIFO order
  """

  def __init__(self, size, method):
    self._que = deque()
    self._size = size
    self._method = method

  def append(self, val):
    if len(self._que) >= self._size:
      self._que.pop()
    self._que.appendleft(val)

  @property
  def valid(self):
    return len(self._que) == self._size

  @property
  def value(self):
    # TODO: consider caching the value to speed up query
    return self._method(self._que)

class GroundDetector:
  """
  GroundDetector uses readings of the link states obtained either directly from
  the simulator via the gazebo/link_states topic or computed through TF2 package
  and implements ground detection by observing a change in movement of the lander
  arm as it descends to touch the ground.
  """

  def __init__(self):
    self._buffer = tf2_ros.Buffer()
    self._listener = tf2_ros.TransformListener(self._buffer)
    self.reset()
    self._check_ground_method = self._tf_2_method

    if self._check_ground_method == self._gazebo_link_states_method:
      rospy.Subscriber("/gazebo/link_states", LinkStates, self._handle_link_states)
      self._query_ground_position_method = lambda: self._tf_lookup_position(rospy.Duration(10))
    else:
      self._query_ground_position_method = lambda: self._last_position

  def reset(self):
    """ Reset the state of the ground detector for another round """
    self._last_position, self._last_time = None, None
    self._ground_detected = False
    self._trending_velocity = SlidingWindow(3, np.mean)

  def _check_condition(self, new_position):
    if self._last_position is None:
      self._last_position = np.array([new_position.x, new_position.y, new_position.z])
      self._last_time = time.time()
      return False
    current_position = np.array([new_position.x, new_position.y, new_position.z])
    current_time = time.time()
    delta_d = current_position - self._last_position
    delta_t = current_time - self._last_time
    self._last_position, self._last_time = current_position, current_time
    self._trending_velocity.append(delta_d[2] / delta_t)
    return self._trending_velocity.value > GROUND_DETECTION_THRESHOLD if self._trending_velocity.valid else False

  def _handle_link_states(self, data):
    # if ground is found ignore further readings until the detector has been reset
    if self._ground_detected:
      return

    try:
      idx = data.name.index("lander::l_scoop_tip")
    except ValueError:
      rospy.logerr_throttle(1, "GroundDetector: lander::l_scoop_tip not found in link_states")
      return

    self._ground_detected = self._check_condition(data.pose[idx].position)

  def _gazebo_link_states_method(self):
    return self._ground_detected

  def _tf_lookup_position(self, timeout=rospy.Duration(0.0)):
    try:
      t = self._buffer.lookup_transform(
          "base_link", "l_scoop_tip", rospy.Time(), timeout)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      rospy.logwarn("GroundDetector: tf2 raised an exception")
      return None
    return t.transform.translation

  def _tf_2_method(self):
    return self._check_condition(self._tf_lookup_position())

  @property
  def ground_position(self):
    """
    Use this method right after ground has been detected to get ground position
    with reference to the base_link
    """
    return self._query_ground_position_method()

  def detect(self):
    """
    Checks if the robot arm has hit the ground
    :returns: True if ground was detected, False otherwise
    :rtype: boolean
    """
    return self._check_ground_method()