# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import tf2_ros
from geometry_msgs.msg import Point, WrenchStamped

class GroundDetector:
  """GroundDetector uses readings of the force torque sensor to detect when the
  arm has reached the ground (or any downward blocker in general).
  """
  def __init__(self):
    self._detected = False
    self._ft_sensor_sub = rospy.Subscriber('/ft_sensor_dist_pitch',
                                           WrenchStamped,
                                           self._ft_sensor_cb)

  def _ft_sensor_cb(self, msg):
    """Checks if force threshold has been reached
    msg -- Instance of geometry_msgs.msg.WrenchStamped
    """
    # TODO: could this be made more robust by looking at force magnitude instead
    #       of just the x-component?
    FORCE_X_THRESHOLD = -100.0
    if msg.wrench.force.x < FORCE_X_THRESHOLD:
      self._detected = True

  @property
  def ground_detected(self):
    return self._detected
