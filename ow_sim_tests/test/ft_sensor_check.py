#!/usr/bin/env python

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import sys
import rospy
import roslib
import unittest
from geometry_msgs.msg import Wrench, Vector3
from owl_msgs.msg import ArmEndEffectorForceTorque
import moveit_commander
import action_testing

## OW-934
# from ow_lander.msg import UnstowAction, UnstowGoal

## OW-933
from owl_msgs.msg import ArmUnstowAction, ArmUnstowGoal

PKG = 'ow_sim_tests'
roslib.load_manifest(PKG)


# a class that monitors minimum reported frame rate by a gazebo simulation
class FT_Sensor_Check(unittest.TestCase):

  # expected values for testing ft sensor against [force vector[3], torque vector[3]]
  _FT_TARGET_STOW   = Wrench(Vector3(-4.25, -14.74, -0.15), Vector3(-0.13, 0.16, -1.70))
  _FT_TARGET_UNSTOW = Wrench(Vector3(15.20, 0.31, -1.47), Vector3(0.08, 0.15, -1.11))

  def __init__(self, *args, **kwargs):
    unittest.TestCase.__init__(self, *args, **kwargs)
    rospy.init_node("ft_sensor_check", anonymous=True)
    self._test_duration = rospy.get_param("/ft_sensor_check/test_duration")
    self._ft_wrench_msg = None
    self._arm_end_effector_force_torque_sensor_sub = rospy.Subscriber(
      "/arm_end_effector_force_torque", ArmEndEffectorForceTorque, self._arm_end_effector_force_torque_callback)
    moveit_commander.roscpp_initialize(sys.argv)
    self._robot = moveit_commander.RobotCommander()
    self._joint_names = self._robot.get_joint_names("arm")
    self._arm_move_group = moveit_commander.MoveGroupCommander(
        "arm", wait_for_servers=20.0)

    # proceed with test only when ros clock has been initialized
    while rospy.get_time() == 0:
      rospy.sleep(0.1)

  def _arm_end_effector_force_torque_callback(self, msg):
    self._ft_wrench_msg = msg.value

  def test_01_check_ft_sensor_stowed(self):

    test_start_time = rospy.get_time()
    elapsed = 0
    while not rospy.is_shutdown() and elapsed < self._test_duration:
      elapsed = rospy.get_time() - test_start_time
      rospy.sleep(0.1)

    self.assertIsNotNone(self._ft_wrench_msg, "no ft sensor message was received")
    self.assertAlmostEqual(self._ft_wrench_msg.force.x, self._FT_TARGET_STOW.force.x, delta=1.0)
    self.assertAlmostEqual(self._ft_wrench_msg.force.y, self._FT_TARGET_STOW.force.y, delta=1.0)
    self.assertAlmostEqual(self._ft_wrench_msg.force.z, self._FT_TARGET_STOW.force.z, delta=1.0)
    self.assertAlmostEqual(self._ft_wrench_msg.torque.x, self._FT_TARGET_STOW.torque.x, delta=1.0)
    self.assertAlmostEqual(self._ft_wrench_msg.torque.y, self._FT_TARGET_STOW.torque.y, delta=1.0)
    self.assertAlmostEqual(self._ft_wrench_msg.torque.z, self._FT_TARGET_STOW.torque.z, delta=1.0)

  def test_02_unstow(self):
    action_testing.test_action_noyaml(
     self, 'ArmUnstow', ArmUnstowAction, ArmUnstowGoal, 30)

  def test_03_check_ft_sensor_unstowed(self):
    test_start_time = rospy.get_time()
    elapsed = 0
    while not rospy.is_shutdown() and elapsed < self._test_duration:
      elapsed = rospy.get_time() - test_start_time
      rospy.sleep(0.1)

    self.assertIsNotNone(self._ft_wrench_msg, "no ft sensor message was received")
    self.assertAlmostEqual(self._ft_wrench_msg.force.x, self._FT_TARGET_UNSTOW.force.x, delta=1.0)
    self.assertAlmostEqual(self._ft_wrench_msg.force.y, self._FT_TARGET_UNSTOW.force.y, delta=1.0)
    self.assertAlmostEqual(self._ft_wrench_msg.force.z, self._FT_TARGET_UNSTOW.force.z, delta=1.0)
    self.assertAlmostEqual(self._ft_wrench_msg.torque.x, self._FT_TARGET_UNSTOW.torque.x, delta=1.0)
    self.assertAlmostEqual(self._ft_wrench_msg.torque.y, self._FT_TARGET_UNSTOW.torque.y, delta=1.0)
    self.assertAlmostEqual(self._ft_wrench_msg.torque.z, self._FT_TARGET_UNSTOW.torque.z, delta=1.0)

if __name__ == '__main__':
  import rostest
  rostest.rosrun(PKG, 'ft_sensor_check', FT_Sensor_Check)
