# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

"""Defines a fault messages interface to the OceanWATERS simulator. Integrates
communication mechanism between ow_faults_detection package and the ow_lander package.
"""

import rospy
from ow_lander.common import Singleton
from owl_msgs.msg import ArmFaultsStatus

class FaultsInterface(metaclass = Singleton):
  """Communication not directly publish the arm_faults_status, instead, publish the arm_faults_internal
  and let ow_fault_detector handle the messages.
  """
  def __init__(self):
    self._arm_faults_status = ArmFaultsStatus.NONE
    self._arm_faults_internal_pub = rospy.Publisher('arm_faults_internal', 
                                                   ArmFaultsStatus, 
                                                   queue_size = 10, 
                                                   latch = True)

  def set_arm_faults_flag(self, flags):
    self._arm_faults_status |= flags
    self._publish()

  def reset_arm_faults_flags(self):
    self._arm_faults_status = ArmFaultsStatus.NONE
    self._publish()

  def _publish(self):
    self._arm_faults_internal_pub.publish(value = self._arm_faults_status)