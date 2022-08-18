#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import tf2_ros
from geometry_msgs.msg import Point, WrenchStamped

FORCE_X_THRESHOLD = -100.0 # TODO: Optimize for sensor noise as part of OCEANWATERS-617

class GroundDetector:
  """
  GroundDetector uses readings of the force torque sensor to detect when the arm
  has reached the ground (or any downward blocker in general).
  """

  def __init__(self):
    self._buffer = tf2_ros.Buffer()
    self._listener = tf2_ros.TransformListener(self._buffer)

  def _tf_lookup_position(self, timeout=rospy.Duration(0.0)):
    """
    :type timeout: rospy.rostime.Duration
    """
    try:
      t = self._buffer.lookup_transform(
          "base_link", "l_scoop_tip", rospy.Time(), timeout)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      rospy.logwarn("GroundDetector: tf2 raised an exception")
      return None
    return t.transform.translation

  def _ft_sensor_dist_pitch_callback(self, msg):
    """
    :type msg: WrenchStamped
    """
    # rospy.logdebug("msg.wrench.force.x = %f", msg.wrench.force.x)
    if msg.wrench.force.x < FORCE_X_THRESHOLD:
      self._ground_detected = True
      self._dist_pitch_ft_sensor_sub.unregister()

  def reset(self):
    self._ground_detected = False
    self._dist_pitch_ft_sensor_sub = rospy.Subscriber(
      "/ft_sensor_dist_pitch", WrenchStamped, self._ft_sensor_dist_pitch_callback)

  @property
  def ground_position(self):
    """
    Use this method right after ground has been detected to get ground position
    with reference to the base_link.
    """
    t = self._tf_lookup_position(rospy.Duration(10))
    return Point(t.x, t.y, t.z)

  def detect(self):
    """
    :returns: True when ground is detected, False otherwise.
    """
    return self._ground_detected
