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
    test_start_time = time.time()
    arm_activity = rospy.ServiceProxy(service_name, service_type)
    success = arm_activity() if service_args is None else arm_activity(*service_args)
    self.assertTrue(success, "submitted request to " + service_name)
    goal_state_achieved = False

    while not rospy.is_shutdown() and \
        (time.time() - test_start_time < test_duration) and \
        not goal_state_achieved:

      joints_state = self._arm_move_group.get_current_joint_values()
      normalized_joints_state = [ self._normalize_angle(e) for e in joints_state ]
      goal_state_achieved = self._all_close(joints_goal, normalized_joints_state)
      time.sleep(0.2)

    self.assertTrue(goal_state_achieved)

  def test_1_unstow(self):
    joints_target = self._arm_move_group.get_named_target_values("arm_unstowed")
    joints_goal = self._map_named_joint_target_to_list(joints_target)
    self._test_activity(
      service_name = '/arm/unstow',
      service_type = Unstow,
      joints_goal = joints_goal,
      test_duration = rospy.get_param("/arm_check/unstow_duration"))

  def test_2_stow(self):
    joints_target = self._arm_move_group.get_named_target_values("arm_stowed")
    joints_goal = self._map_named_joint_target_to_list(joints_target)
    self._test_activity(
      service_name = '/arm/stow',
      service_type = Stow,
      joints_goal = joints_goal,
      test_duration = rospy.get_param("/arm_check/stow_duration"))

  def test_3_deliver_sample(self):
    self._test_activity(
      service_name = '/arm/deliver_sample',
      service_type = DeliverSample,
      joints_goal = ['0.74', '1.34', '1.85', '-0.28', '-2.87', '2.37'],
      test_duration = rospy.get_param("/arm_check/deliver_sample_duration"),
      service_args = [True, 0, 0, 0])

if __name__ == '__main__':
  import rostest
  rostest.rosrun(PKG, 'arm_check', ArmCheck)
