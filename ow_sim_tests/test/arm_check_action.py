#!/usr/bin/env python

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import sys
import rospy
import roslib
import unittest
import moveit_commander
import actionlib
from geometry_msgs.msg import Point
import ow_lander.msg
from math import sqrt

PKG = 'ow_sim_tests'
roslib.load_manifest(PKG)

"""
Computes the 3D distance between two geometry_msgs.msg Points
"""
def distance(p1, p2):
  v = Point(p2.x - p1.x,
            p2.y - p1.y,
            p2.z - p1.z)
  return sqrt(v.x*v.x + v.y*v.y + v.z*v.z)

class ArmCheckAction(unittest.TestCase):

  def __init__(self, *args, **kwargs):
    unittest.TestCase.__init__(self, *args, **kwargs)
    rospy.init_node("arm_check_action_test", anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    self._robot = moveit_commander.RobotCommander()
    self._joint_names = self._robot.get_joint_names("arm")
    self._arm_move_group = moveit_commander.MoveGroupCommander(
        "arm", wait_for_servers=20.0)

    # proceed with test only when ros clock has been initialized
    while rospy.get_time() == 0:
      rospy.sleep(0.1)


  def _assert_nothing(self):
    pass

  """
  Returns true if an action is done
  @param action_client: ROS action client object
  """
  def _is_action_done(self, action_client):
    return action_client.simple_state == actionlib.SimpleGoalState.DONE

  """
  Asserts two Points are near each
  @param p1: First point
  @param p2: Second point
  @param delta: Distance under which points are considered "near"
  @param msg: Assert message
  """
  def _assert_point_is_near(self, p1, p2, delta, msg):
    self.assertLessEqual(distance(p1, p2), delta, msg)

  """
  Calls an action asynchronously allowing checks to occur during its execution.
  @param action_name: Name of action to be called
  @param action: ROS action object
  @param goal: Goal object that kwargs populates
  @param max_duration: Max time allotted to action in seconds
  @param action_fail_condition: A function repeatedly called during action
         execution that asserts requirements for a successful action operation
  @param expected_final: Final arm position to be compared with action result
         (default: None)
  @param server_timeout: Time the action server is waited for (default: 10.0)
  @param condition_check_interval: The interval in seconds between calls to
         action_fail_condition (default: 0.2)
  @param expected_final_tolerance: allowed distance between result.final and
         expected_final (default: 0.02)
  @kwargs: Any properties of goal
  """
  def _test_action(self, action_name, action, goal, max_duration,
                   action_fail_condition,
                   expected_final = None,
                   server_timeout = 10.0,
                   condition_check_interval=0.2,
                   expected_final_tolerance = 0.02,
                   **kwargs):

    client = actionlib.SimpleActionClient(action_name, action)
    connected = client.wait_for_server(timeout=rospy.Duration(server_timeout))
    self.assertTrue(
      connected,
      "Timeout exceeded waiting for %s action server" % action_name
    )

  # populate goal parameters using kwargs
    parameter_list = ""
    for key, value in kwargs.items():
      if hasattr(goal, key):
        setattr(goal, key, value)
        parameter_list += "\n%s : %s" % (key, str(value))
      else:
        self.fail(
          "%s action does not contain parameter %s" % (action_name, key)
        )

    client.send_goal(goal)

    rospy.loginfo(
      "\n===%s action goal sent===%s" % (action_name, parameter_list)
    )

    # monitor for failed conditions during action execution
    start = rospy.get_time()
    elapsed = 0.0
    while not self._is_action_done(client) and elapsed < max_duration:
      action_fail_condition()
      rospy.sleep(condition_check_interval)
      elapsed = rospy.get_time() - start

    self.assertLess(
      elapsed, max_duration,
      "Timeout reached waiting for %s action to finish!" % action_name
    )

    result = client.get_result()
    rospy.loginfo(
      "\n===%s action goal sent===%s" % (action_name, parameter_list)
    )
    # verify action ended where we expected
    if expected_final is not None:

      self._assert_point_is_near(
        result.final, expected_final, expected_final_tolerance,
        "Arm did not complete the %s action in the position expected! Fianl position is x =%s  y =%s  z =%s" 
          % (action_name, result.final.x, result.final.y, result.final.z)
      )
      

    return result

  def test_01_unstow(self):
    UNSTOW_MAX_DURATION = 30.0
    unstow_result = self._test_action(
      'Unstow',
      ow_lander.msg.UnstowAction,
      ow_lander.msg.UnstowGoal(),
      UNSTOW_MAX_DURATION,
      self._assert_nothing,
      expected_final = Point(1.7419, 0.2396, -6.5904),
      expected_final_tolerance = 0.5, # unstow requires a really high tolerance
      server_timeout = 25.0
    )

  def test_02_guarded_move(self):
    GUARDED_MAX_DURATION = 60.0
    guarded_move_result = self._test_action(
      'GuardedMove',
      ow_lander.msg.GuardedMoveAction,
      ow_lander.msg.GuardedMoveGoal(),
      GUARDED_MAX_DURATION,
      self._assert_nothing,
      expected_final =  Point(2.0324, -0.1012, -0.1585),
      start = Point(2.0, 0.0, 0.3),
      normal = Point(0.0, 0.0, 1.0),
      search_distance = 0.5
    )

  def test_03_unstow(self):
    UNSTOW_MAX_DURATION = 30.0
    unstow_result = self._test_action(
      'Unstow',
      ow_lander.msg.UnstowAction,
      ow_lander.msg.UnstowGoal(),
      UNSTOW_MAX_DURATION,
      self._assert_nothing,
      expected_final = Point(1.7419, 0.2396, -6.5904),
      expected_final_tolerance = 0.5, # unstow requires a really high tolerance
      server_timeout = 25.0
    )

  def test_04_grind(self):
    GRIND_MAX_DURATION = 80.0 
    grind_result = self._test_action(
      'Grind',
      ow_lander.msg.GrindAction,
      ow_lander.msg.GrindGoal(),
      GRIND_MAX_DURATION,
      self._assert_nothing,
      expected_final = Point(1.4720, -0.1407, -6.7400),
      x_start         = 1.65,
      y_start         = 0.0,
      depth           = 0.15,
      length          = 0.7,
      parallel        = True,
      ground_position = -0.155
    )

  def test_05_dig_circular(self):
    DIG_CIRCULAR_MAX_DURATION = 60.0
    dig_circular_result = self._test_action(
      'DigCircular',
      ow_lander.msg.DigCircularAction,
      ow_lander.msg.DigCircularGoal(),
      DIG_CIRCULAR_MAX_DURATION,
      self._assert_nothing,
      expected_final = Point(1.6499, -0.1219, -7.3220),
      x_start         = 1.65,
      y_start         = 0.0,
      depth           = 0.01,
      parallel        = False,
      ground_position = -0.155
    )

  def test_06_grind(self):
    GRIND_MAX_DURATION = 80.0 
    grind_result = self._test_action(
      'Grind',
      ow_lander.msg.GrindAction,
      ow_lander.msg.GrindGoal(),
      GRIND_MAX_DURATION,
      self._assert_nothing,
      expected_final = Point(1.4720, -0.1407, -6.7400),
      x_start         = 1.65,
      y_start         = 0.0,
      depth           = 0.15,
      length          = 0.7,
      parallel        = True,
      ground_position = -0.155
    )

  def test_07_dig_linear(self):
    DIG_LINEAR_MAX_DURATION = 110.0
    dig_linear_result = self._test_action(
      'DigLinear',
      ow_lander.msg.DigLinearAction,
      ow_lander.msg.DigLinearGoal(),
      DIG_LINEAR_MAX_DURATION,
      self._assert_nothing,
      expected_final = Point(2.2404, -0.0121, -7.0493),
      x_start         = 1.46,
      y_start         = 0.0,
      depth           = 0.01,
      length          = 0.1,
      ground_position = -0.155
    )

  def test_08_deliver_sample(self):
    DELIVER_MAX_DURATION = 200.0
    deliver_result = self._test_action(
      'Deliver',
      ow_lander.msg.DeliverAction,
      ow_lander.msg.DeliverGoal(),
      DELIVER_MAX_DURATION,
      self._assert_nothing,
      expected_final = Point(0.5562, -0.2135, -6.3511),
    )
  
  def test_09_unstow(self):
    UNSTOW_MAX_DURATION = 30.0
    unstow_result = self._test_action(
      'Unstow',
      ow_lander.msg.UnstowAction,
      ow_lander.msg.UnstowGoal(),
      UNSTOW_MAX_DURATION,
      self._assert_nothing,
      expected_final = Point(1.7419, 0.2396, -6.5904),
      expected_final_tolerance = 0.5, # unstow requires a really high tolerance
      server_timeout = 25.0    
    )

  def test_10_stow(self):
    STOW_MAX_DURATION = 30.0
    stow_result = self._test_action(
      'Stow',
      ow_lander.msg.StowAction,
      ow_lander.msg.StowGoal(),
      STOW_MAX_DURATION,
      self._assert_nothing,
      expected_final = Point(0.7071, -0.4770, -6.5930),
    )


if __name__ == '__main__':
  import rostest
  rostest.rosrun(PKG, 'arm_check_action', ArmCheckAction)
