#!/usr/bin/env python

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import sys
import rospy
import roslib
import unittest
import time
from ow_lander.srv import *
import moveit_commander

PKG = 'ow_sim_tests'
roslib.load_manifest(PKG)

# a class that monitors minimum reported frame rate by a gazebo simulation
class ArmCheck(unittest.TestCase):

  def __init__(self, *args, **kwargs):
    unittest.TestCase.__init__(self, *args, **kwargs)
    rospy.init_node("arm_check_test", anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    self._robot = moveit_commander.RobotCommander()
    self._joint_names = self._robot.get_joint_names("arm")
    self._arm_move_group = moveit_commander.MoveGroupCommander(
        "arm", wait_for_servers=20.0)

    # proceed with test only when ros clock has been initialized
    while rospy.get_time() == 0:
      rospy.sleep(0.1)

  def _map_named_joint_target_to_list(self, joints_target):
    return [joints_target[n] for n in self._joint_names]

  def _normalize_angle(self, angle):
    """
    Convenience method to nomrlaize angles to the range [-pi, pi]
    @param angle: An angle that might need to be normalized
    @returns: the normalized value of the angle
    """
    from math import pi
    angle =  angle % (2*pi)
    angle = (angle + 2*pi) % (2*pi)
    if angle > pi:
        angle -= 2*pi
    return angle

  def _all_close(self, goal, actual, tolerance=0.01):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param goal: A list of floats
    @param actual: A list of floats
    @param tolerance: A float
    @returns: True if all values satify the condition, otherwise False.
    """
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False
    return True

  def _test_activity(self,
    service_name, service_type, joints_goal, test_duration, service_args = None):
    """
    A helper method that does validates a any arm operation
    @param service_name: Name of the ros service to be invoked
    @param service_type: Type of the service
    @param joints_goal: Expected joints values when the activity is completed
    @param test_duration: How long the operation needs to complete
    @param service_args: [optional] Args to be passed to the service when invoked
    """

    rospy.wait_for_service(service_name)
    success = False
    test_start_time = rospy.get_time()
    arm_activity = rospy.ServiceProxy(service_name, service_type)
    success = arm_activity() if service_args is None else arm_activity(*service_args)
    self.assertTrue(success, "submitted request to " + service_name)
    goal_state_achieved = False

    rospy.loginfo("testing service: " + service_name)

    elapsed = 0
    while not rospy.is_shutdown() and \
        elapsed < test_duration and \
        not goal_state_achieved:

      joints_state = self._arm_move_group.get_current_joint_values()
      normalized_joints_state = [ self._normalize_angle(e) for e in joints_state ]
      goal_state_achieved = self._all_close(joints_goal, normalized_joints_state)
      rospy.sleep(0.2)
      elapsed = rospy.get_time() - test_start_time

    rospy.loginfo("{} service reported: {}".format(service_name, goal_state_achieved))
    self.assertTrue(elapsed < test_duration, "arm operation timed-out! condition: {} < {}".format(elapsed, test_duration))
    self.assertTrue(goal_state_achieved, "expected joint states don't match!")

  def test_01_unstow(self):
    joints_target = self._arm_move_group.get_named_target_values("arm_unstowed")
    joints_goal = self._map_named_joint_target_to_list(joints_target)
    self._test_activity(
      service_name = '/arm/unstow',
      service_type = Unstow,
      joints_goal = joints_goal,
      test_duration = 30)

  def test_02_guarded_move(self):
    self._test_activity(
      service_name = '/arm/guarded_move',
      service_type = GuardedMove,
      joints_goal = [-0.02, 0.68, -1.39, 0.75, -0.01, -0.053],
      test_duration = 45,
      service_args = [True, 0, 0, 0, 0, 0, 0, 0])

  def test_03_unstow(self):
    joints_target = self._arm_move_group.get_named_target_values("arm_unstowed")
    joints_goal = self._map_named_joint_target_to_list(joints_target)
    self._test_activity(
      service_name = '/arm/unstow',
      service_type = Unstow,
      joints_goal = joints_goal,
      test_duration = 30)

  def test_04_grind(self):
    self._test_activity(
      service_name = '/arm/grind',
      service_type = Grind,
      joints_goal = [-0.12, 1.91, -2.33, 0.40, -2.10, 0.37],
      test_duration = 90,
      service_args = [True, 0, 0, 0, 0, False, 0])

  def test_05_dig_circular(self):
    self._test_activity(
      service_name = '/arm/dig_circular',
      service_type = DigCircular,
      joints_goal = [-0.03, 0.96, -1.64, 1.66, 0.0, 1.57],
      test_duration = 100,
      service_args = [True, 0, 0, 0, False, 0])

  def test_06_grind(self):
    self._test_activity(
      service_name = '/arm/grind',
      service_type = Grind,
      joints_goal = [-0.13, 2.14, -2.46, 0.30, -2.10, 1.56],
      test_duration = 100,
      service_args = [False, 1.55, 0, 0.15, 0.85, True, -0.155])

  def test_07_dig_linear(self):
    self._test_activity(
      service_name = '/arm/dig_linear',
      service_type = DigLinear,
      joints_goal = [-0.04, 0.55, -1.17, 1.57, 0.0,1.57],
      test_duration = 120,
      service_args = [True, 0, 0, 0, 0, 0])

  def test_08_deliver_sample(self):
    self._test_activity(
      service_name = '/arm/deliver_sample',
      service_type = DeliverSample,
      joints_goal = [0.98, 1.78, 0.96,1.98, 2.55,1.58],
      test_duration = 70,
      service_args = [True, 0, 0, 0])

  def test_09_unstow(self):
    joints_target = self._arm_move_group.get_named_target_values("arm_unstowed")
    joints_goal = self._map_named_joint_target_to_list(joints_target)
    self._test_activity(
      service_name = '/arm/unstow',
      service_type = Unstow,
      joints_goal = joints_goal,
      test_duration = 30)

  def test_10_stow(self):
    joints_target = self._arm_move_group.get_named_target_values("arm_stowed")
    joints_goal = self._map_named_joint_target_to_list(joints_target)
    self._test_activity(
      service_name = '/arm/stow',
      service_type = Stow,
      joints_goal = joints_goal,
      test_duration = 25)

if __name__ == '__main__':
  import rostest
  rostest.rosrun(PKG, 'arm_check', ArmCheck)
