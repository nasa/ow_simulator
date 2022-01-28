#!/usr/bin/env python

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import roslib
import actionlib
import unittest

from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point

import ow_lander.msg

PKG = 'ow_sim_tests'
roslib.load_manifest(PKG)

ACTION_SERVER_WAIT_TIMEOUT = rospy.Duration(10.0)

ACCEPTABLE_POSE_POINT_VARIANCE = 0.005

class TerrainInteraction(unittest.TestCase):

  def __init__(self, *args, **kwargs):
    unittest.TestCase.__init__(self, *args, **kwargs)
    rospy.init_node('terrain_interaction_test', anonymous=True)

    # proceed with test only when ros clock has been initialized
    while rospy.get_time() == 0:
      rospy.sleep(0.1)

  """
  Calls an action asynchronously allowing checks to occur during its execution.
  @param action_name: Name of action to be called
  @param action: ROS action object
  @param goal:  Goal object that kwargs populates
  @kwargs: Any properties of goal
  """
  def _start_action(self, action_name, action, goal, timeout, **kwargs):
    client = actionlib.SimpleActionClient(action_name, action)
    connected = client.wait_for_server(timeout=timeout)
    self.assertTrue(connected, "Timeout exceeded waiting for %s action server" % action_name)
    client.wait_for_server()

    # populate goal parameters using kwargs
    parameter_list = ""
    for key, value in kwargs.items():
      if hasattr(goal, key):
        setattr(goal, key, value)
        parameter_list += "\n%s : %s" % (key, str(value))
      else:
        self.fail("%s action does not contain parameter %s" % (action_name, key))

    client.send_goal(goal)
    rospy.loginfo("\n===%s Action Goal Parameters Sent===%s" % (action_name, parameter_list))

    return client

  """
  Checks if an action is done
  @param action_client: ROS action client object
  """
  def _is_action_done(self, action_client):
    return action_client.simple_state == actionlib.SimpleGoalState.DONE

  def assert_point_almost_equals(self, p1, p2, delta=ACCEPTABLE_POSE_POINT_VARIANCE):
    self.assertAlmostEqual(p1.x, p2.x, delta=delta)
    self.assertAlmostEqual(p1.y, p2.y, delta=delta)
    self.assertAlmostEqual(p1.z, p2.z, delta=delta)

  def test_01_grind(self):

    GRIND_ACTIVITY_TIMEOUT = 60.0

    grind_client = self._start_action(
      'Grind',
      ow_lander.msg.GrindAction,
      ow_lander.msg.GrindGoal(),
      ACTION_SERVER_WAIT_TIMEOUT,
      x_start         = 1.65,
      y_start         = 0.0,
      depth           = 0.05,
      length          = 0.6,
      parallel        = True,
      ground_position = -0.155
    )

    """
    Verify during execution:
      1. No regolith models are spawned during the operation.
    """
    condition_violated = False
    def condition_test(message):
      nonlocal condition_violated
      condition_violated = "regolith_0" in message.name

    subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, condition_test)
    start_time = rospy.get_time()
    elapsed = 0
    while not self._is_action_done(grind_client) and \
          elapsed < GRIND_ACTIVITY_TIMEOUT and \
          not condition_violated:
      # ensure no regolith models are spawned
      rospy.sleep(0.2)
      elapsed = rospy.get_time() - start_time
    subscriber.unregister()
    self.assertTrue(elapsed < GRIND_ACTIVITY_TIMEOUT, "Timeout reached waiting for Grind action to finish!")
    self.assertFalse(condition_violated, "Regolith model spawned during the grind operation!")

    """
    Verify post execution:
      1. Action result shows is where we expect it
      2. Terrain has changed shape
    """
    grind_result = grind_client.get_result()
    expected_final = Point(1.4715559423041809, -0.14057547008097895, -6.739779746454913)
    self.assert_point_almost_equals(grind_result.final, expected_final)

    # TODO: find way to check if collision terrain has changed

if __name__ == '__main__':
  import rostest
  rostest.rosrun(PKG, 'terrain_interaction', TerrainInteraction)
