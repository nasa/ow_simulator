# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

"""Defines an interface for actions to register power consumption that will be
sent to power_system_node for processing and inclusion into the overall power
load.
"""

import rospy
from ow_lander.common import Singleton
from std_msgs.msg import Float64

class PowerInterface(metaclass = Singleton):

  def __init__(self):
    self._power_loads = dict()
    self._electrical_power_internal_pub = rospy.Publisher(
      '/electrical_power', Float64, queue_size = 10, latch = True
    )

  def set_power_load(self, key, power):
    self._power_loads[key] = power
    self._publish()

  def reset_power_load(self, key):
    if not key in self._power_loads:
      rospy.logwarn(
        f"The key, {key}, does not have a power associated with it."
      )
      return
    del self._power_loads[key]
    self._publish()

  def _publish(self):
    self._electrical_power_internal_pub.publish(
      data = sum(self._power_loads.values())
    )
