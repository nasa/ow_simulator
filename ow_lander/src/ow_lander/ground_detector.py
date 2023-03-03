# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

from math import sqrt

import rospy
import tf2_ros
from geometry_msgs.msg import Point
from owl_msgs.msg import ArmEndEffectorForceTorque

def _magnitude(vec):
  return sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z)

class FTSensorThresholdMonitor:

  def __init__(self, force_threshold=None, torque_threshold=None):
    self._ft_sensor_sub = rospy.Subscriber('/arm_end_effector_force_torque',
                                           ArmEndEffectorForceTorque,
                                           self._ft_sensor_cb)
    self._force_threshold = force_threshold
    self._torque_threshold = torque_threshold
    self._force = 0
    self._torque = 0

  def _ft_sensor_cb(self, msg):
    wrench = msg.value
    if self.is_force_monitor(): self._force = _magnitude(wrench.force)
    if self.is_torque_monitor(): self._torque = _magnitude(wrench.torque)
    if self.threshold_breached():
      self._ft_sensor_sub.unregister()

  def is_force_monitor(self):
    return self._force_threshold is not None

  def is_torque_monitor(self):
    return self._torque_threshold is not None

  def force_threshold_breached(self):
    return self.is_force_monitor() and self._force >= self._force_threshold

  def torque_threshold_breached(self):
    return self.is_torque_monitor() and self._torque >= self._torque_threshold

  def threshold_breached(self):
    return self.force_threshold_breached() or self.torque_threshold_breached()

  def get_force(self):
    return self._force if self.is_force_monitor() else None

  def get_torque(self):
    return self._torque if self.is_torque_monitor() else None


# DEPRECATED: Only supports GuardedMove, which is itself deprecated
class GroundDetector:
  """GroundDetector uses readings of the force torque sensor to detect when the
  arm has reached the ground (or any downward blocker in general).
  """
  def __init__(self, reference_frame, poker_link):
    self._detected = False
    self._ft_sensor_sub = rospy.Subscriber('/arm_end_effector_force_torque',
                                           ArmEndEffectorForceTorque,
                                           self._ft_sensor_cb)
    self._buffer = tf2_ros.Buffer()
    self._listener = tf2_ros.TransformListener(self._buffer)
    self._frame = reference_frame
    self._link = poker_link

  def _ft_sensor_cb(self, msg):
    """Checks if force threshold has been reached
    msg -- Instance of ArmEndEffectorForceTorque.msg.value, a geometry_msgs/Wrench
            datatype with child datatypes Vector3 force and Vector3 torque
    """
    # TODO: could this be made more robust by looking at force magnitude instead
    #       of just the x-component?
    FORCE_X_THRESHOLD = -100.0
    if msg.value.force.x < FORCE_X_THRESHOLD:
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
