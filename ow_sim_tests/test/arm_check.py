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
    self._test_duration = rospy.get_param("/arm_check/test_duration")
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
    service_name, service_type, joints_goal):
    """
    A helper method that does validates a any arm operation
    @param service_name:
    @param service_type:
    @param joints_goal:
    """

    rospy.wait_for_service(service_name)
    success = False
    self._test_start_time = time.time()

    try:
      arm_activity = rospy.ServiceProxy(service_name, service_type)
      success = arm_activity()
    except rospy.ServiceException as e:
      rospy.logerr("{} threw an error: {}".format(service_name, e))

    self.assertTrue(success, "submitted request to " + service_name)

    goal_state_achieved = False

    while not rospy.is_shutdown() and \
        (time.time() - self._test_start_time < self._test_duration) and \
        not goal_state_achieved:

      joints_state = self._arm_move_group.get_current_joint_values()
      rospy.loginfo("joints_goal = {}".format(joints_goal))
      normalized_joints_state = [ self._normalize_angle(e) for e in joints_state ]
      formatted_joints_state = [ '%.2f' % e for e in normalized_joints_state ]
      rospy.loginfo("joints_state = {}".format(formatted_joints_state))
      goal_state_achieved = self._all_close(joints_goal, normalized_joints_state)
      rospy.loginfo("achieved? {}".format(goal_state_achieved))
      time.sleep(0.2)

    self.assertTrue(goal_state_achieved)

  def test_unstow(self):
    joints_target = self._arm_move_group.get_named_target_values("arm_unstowed")
    joints_goal = self._map_named_joint_target_to_list(joints_target)
    self._test_activity(
      service_name = '/arm/unstow', service_type = Unstow, joints_goal = joints_goal)

if __name__ == '__main__':
  import rostest
  rostest.rosrun(PKG, 'test_unstow', ArmCheck)
