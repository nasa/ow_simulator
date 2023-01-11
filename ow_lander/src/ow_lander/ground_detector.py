# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

from math import sqrt

import rospy
import tf2_ros
from geometry_msgs.msg import Point, WrenchStamped

def _magnitude(vec):
  return sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z)

class FTSensorThresholdMonitor:

  def __init__(self, force_threshold, torque_threshold):
    self._ft_sensor_sub = rospy.Subscriber('/ft_sensor_dist_pitch',
                                           WrenchStamped,
                                           self._ft_sensor_cb)
    self._force_threshold = force_threshold
    self._torque_threshold = torque_threshold
    self._force = 0
    self._torque = 0

  def _ft_sensor_cb(self, msg):
    wrench = msg.wrench
    self._force = _magnitude(wrench.force)
    self._torque = _magnitude(wrench.torque)
    if self.threshold_breached():
      self._ft_sensor_sub.unregister()

  def force_threshold_breached(self):
    return self._force >= self._force_threshold

  def torque_threshold_breached(self):
    return self._torque >= self._torque_threshold

  def threshold_breached(self):
    return self.force_threshold_breached() or self.torque_threshold_breached()

  def get_force(self):
    return self._force

  def get_torque(self):
    return self._torque


# DEPRECATED: Only supports GuardedMove, which is itself deprecated
class GroundDetector:
  """GroundDetector uses readings of the force torque sensor to detect when the
  arm has reached the ground (or any downward blocker in general).
  """
  def __init__(self, reference_frame, poker_link):
    self._detected = False
    self._ft_sensor_sub = rospy.Subscriber('/ft_sensor_dist_pitch',
                                           WrenchStamped,
                                           self._ft_sensor_cb)
    self._buffer = tf2_ros.Buffer()
    self._listener = tf2_ros.TransformListener(self._buffer)
    self._frame = reference_frame
    self._link = poker_link

  def _ft_sensor_cb(self, msg):
    """Checks if force threshold has been reached
    msg -- Instance of geometry_msgs.msg.WrenchStamped
    """
    # TODO: could this be made more robust by looking at force magnitude instead
    #       of just the x-component?
    FORCE_X_THRESHOLD = -100.0
    if msg.wrench.force.x < FORCE_X_THRESHOLD:
      self._detected = True

  def was_ground_detected(self):
    return self._detected

  def get_ground_position(self):
    TIMEOUT = rospy.Duration(10) # seconds
    try:
      t = self._buffer.lookup_transform(self._frame, self._link, rospy.Time(),
        timeout=TIMEOUT)
    except (tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException) as err:
      rospy.logerr(f"GroundDetector.get_ground_position: {str(err)}")
      return Point()
    return Point(
      t.transform.translation.x,
      t.transform.translation.y,
      t.transform.translation.z
    )
