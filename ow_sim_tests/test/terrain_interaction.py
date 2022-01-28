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

ACTION_SERVER_WAIT_TIMEOUT = rospy.Duration(5.0)

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
    client.wait_for_server(timeout=timeout)

    # use kwargs here
    for key, value in kwargs.items():
      if hasattr(goal, key):
        setattr(goal, key, value)
      else:
        self.fail("%s action does not contain parameter %s" % (action_name, key))
    client.send_goal(goal)
    return client

  """
  Checks if an action is done
  @param action_client: ROS action client object
  """
  def _is_action_done(self, action_client):
    rospy.loginfo("state = %d" % action_client.simple_state)
    return action_client.simple_state == actionlib.SimpleGoalState.DONE

  def test_01_grind(self):

    GRIND_ACTIVITY_TIMEOUT = 60.0

    grind_client = self._start_action(
      "Grind",
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
    self.assertFalse(condition_violated, "Regolith model spawned during the grind operation!")

    """
    Verify post execution:
      1. Action result shows is where we expect it
      2. Terrain has changed shape
    """
    grind_result = grind_client.get_result()
    # TODO: test for final value
    # self.assertEquals(grind_result.final, Point())
    # TODO: find way to check if terrain has changed

if __name__ == '__main__':
  import rostest
  rostest.rosrun(PKG, 'terrain_interaction', TerrainInteraction)
