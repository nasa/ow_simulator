#!/usr/bin/env python

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

from math import sqrt
from copy import copy
import unittest
import rospy
import roslib
import actionlib

from gazebo_msgs.msg import ModelStates, LinkStates, LinkState
from geometry_msgs.msg import Point

import ow_lander.msg

PKG = 'ow_sim_tests'
roslib.load_manifest(PKG)

ACCEPTABLE_POSE_POINT_VARIANCE = 0.01 # meters

GROUND_POSITION = -0.155

SCOOP_LINK_NAME = 'lander::l_scoop'
SAMPLE_DOCK_LINK_NAME = 'lander::lander_sample_dock_link'

REGOLITH_PREFIX = 'regolith_'

"""
Computes the 3D distance between two geometry_msgs.msg Points
"""
def distance(p1, p2):
  v = Point(p2.x - p1.x,
            p2.y - p1.y,
            p2.z - p1.z)
  return sqrt(v.x*v.x + v.y*v.y + v.z*v.z)

"""
Computes the distance in only the X-Y plane between two geometry_msgs.msg Points
"""
def distance_flat_xy(p1, p2):
  v = Point(p2.x - p1.x,
            p2.y - p1.y,
            0)
  return sqrt(v.x*v.x + v.y*v.y)

class TerrainInteraction(unittest.TestCase):

  def __init__(self, *args, **kwargs):
    unittest.TestCase.__init__(self, *args, **kwargs)
    rospy.init_node('terrain_interaction_test', anonymous=True)

    # subscribe to get all gazebo link positions
    self._gz_link_names = []
    self._gz_link_poses = []
    self._link_states_sub = rospy.Subscriber("/gazebo/link_states",
                                             LinkStates,
                                             self._on_link_states)

    # proceed with test only when ros clock has been initialized
    while rospy.get_time() == 0:
      rospy.sleep(0.1)

  """
  Callback function for the link_states topic
  """
  def _on_link_states(self, message):
    self._gz_link_names = message.name
    self._gz_link_poses = message.pose

  """
  Returns true if an action is done
  @param action_client: ROS action client object
  """
  def _is_action_done(self, action_client):
    return action_client.simple_state == actionlib.SimpleGoalState.DONE

  """
  Returns true if model/link name matches the regolith prefix
  @param name: Name of a model/link
  """
  def _is_regolith(self, name):
    return name[:len(REGOLITH_PREFIX)] == REGOLITH_PREFIX

  """
  Returns the position of a givne link
  @param link_name: Name of a valid gazebo link
  """
  def _get_link_position(self, link_name):
    try:
      i = self._gz_link_names.index(link_name)
    except ValueError as e:
      rospy.logwarn(e)
      return None
    else:
      # make a shallow copy
      # otherwise functions that call this can modify positions by reference
      return copy(self._gz_link_poses[i].position)

  """
  Asserts that there are no regolith models present in the Gazebo world
  @param msg: Assert message
  """
  def _assert_regolith_not_present(self, msg="Regolith models spawned at wrong time!"):
    # Verify no regolith models spawn during grind action
    self.assertFalse(
      any([self._is_regolith(name) for name in self._gz_link_names]),
      msg
    )

  """
  Asserts that there are regolith models present in the Gazebo world
  @param msg: Assert message
  """
  def _assert_regolith_present(self, msg="No regolith models spawned!"):
    self.assertTrue(
      any([self._is_regolith(name) for name in self._gz_link_names]),
      msg
    )

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
  Assert that, if regolith exists, it is in the scoop
  """
  def _assert_regolith_is_in_scoop(self):
    # Verify regolith models remain in the scoop after spawning
    scoop_position = self._get_link_position(SCOOP_LINK_NAME)
    if scoop_position is None:
      return
    for i, (name, pose) in enumerate(zip(self._gz_link_names, self._gz_link_poses)):
      if self._is_regolith(name):
        self._assert_point_is_near(
          pose.position, scoop_position, 0.12,
          "Regolith fell out of scoop!\n"
          "regolith name     : %s\n"
          "regolith position : (%.2f, %.2f, %.2f)\n"
          "scoop position    : (%.2f, %.2f, %.2f)"
            % (
                name,
                pose.position.x, pose.position.y, pose.position.z,
                scoop_position.x, scoop_position.y, scoop_position.z
              )
        )

  """
  Asserts regolith remains in scoop until it arrives at the sample dock
  """
  def _assert_regolith_transports_and_delivers(self):
    # while scoop is transitioning to sample dock ensure reoglith remains in
    scoop_position = self._get_link_position(SCOOP_LINK_NAME)
    dock_position = self._get_link_position(SAMPLE_DOCK_LINK_NAME)
    if scoop_position is None or dock_position is None:
      return
    # check if scoop is still transitioning to sample dock
    if distance_flat_xy(scoop_position, dock_position) > 0.2:
      self._assert_regolith_is_in_scoop()


  """
  Calls an action asynchronously allowing checks to occur during its execution.
  @param action_name: Name of action to be called
  @param action: ROS action object
  @param goal: Goal object that kwargs populates
  @param max_duration: Max time allotted to action in seconds
  @param expected_final: Final arm position to be compared with action result
  @param action_fail_condition: A function repeatedly called during action
         execution that asserts requirements for a successful action operation
  @param server_timeout: Time the action server is waited for (default: 10.0)
  @param condition_check_interval: The interval in seconds between calls to
         action_fail_condition (default: 0.2)
  @kwargs: Any properties of goal
  """
  def _test_action(self, action_name, action, goal,
                   max_duration, expected_final, action_fail_condition,
                   server_timeout=10.0,
                   condition_check_interval=0.2,
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
      "\n===%s action completed in %0.3fs===" % (action_name, elapsed)
    )

    # verify action ended where we expected
    final_fail_message = "Arm did not complete the %s action in the position expected!" % action_name
    self.assertAlmostEqual(result.final.x, expected_final.x,
                           delta=ACCEPTABLE_POSE_POINT_VARIANCE,
                           msg=final_fail_message)
    self.assertAlmostEqual(result.final.y, expected_final.y,
                           delta=ACCEPTABLE_POSE_POINT_VARIANCE,
                           msg=final_fail_message)
    self.assertAlmostEqual(result.final.z, expected_final.z,
                           delta=ACCEPTABLE_POSE_POINT_VARIANCE,
                           msg=final_fail_message)

    return result

  """
  Tests an extra deep grind action and asserts no regolith is spawned during the
  operation.
  TODO:
    1. verify terrain has been modified
  """
  def test_01_grind(self):

    GRIND_MAX_DURATION = 60.0 # seconds
    GRIND_EXPECTED_FINAL = Point(1.4715559423041809,
                                 -0.14057547008097895,
                                 -6.739779746454913)

    # call Grind action asynchronously
    grind_result = self._test_action(
      'Grind',
      ow_lander.msg.GrindAction,
      ow_lander.msg.GrindGoal(),
      GRIND_MAX_DURATION,
      GRIND_EXPECTED_FINAL,
      self._assert_regolith_not_present,
      server_timeout = 20.0, # (seconds) first action call needs longer timeout
      x_start         = 1.65,
      y_start         = 0.0,
      depth           = 0.15, # grind deep so terrain is modified
      length          = 0.6,
      parallel        = True,
      ground_position = GROUND_POSITION
    )

  """
  Tests a default dig linear action and asserts regolith does spawn during the
  operation and remains in the scoop until the end of it.
  """
  def test_02_dig_linear(self):

    DIG_LINEAR_MAX_DURATION = 110.0
    DIG_LINEAR_EXPECTED_FINAL = Point(2.2404188541487606,
                                      -0.012052180209644802,
                                      -7.052169181987205)

    # call Grind action asynchronously
    dig_linear_result = self._test_action(
      'DigLinear',
      ow_lander.msg.DigLinearAction,
      ow_lander.msg.DigLinearGoal(),
      DIG_LINEAR_MAX_DURATION,
      DIG_LINEAR_EXPECTED_FINAL,
      self._assert_regolith_is_in_scoop,
      x_start         = 1.46,
      y_start         = 0.0,
      depth           = 0.01,
      length          = 0.1,
      ground_position = GROUND_POSITION
    )

    # verify regolith models spawned
    self._assert_regolith_present()

  """
  Tests a deliver action and asserts that regolith remains in the scoop until
  the final portion when it is dumped into the sample dock. Upon completion of
  the operation it also asserts that regolith models are deleted.
  """
  def test_03_deliver(self):

    DELIVER_MAX_DURATION = 60.0 # TODO: constrain
    DELIVER_EXPECTED_FINAL = Point(0.5562290134759807,
                                   -0.21354780470240303,
                                   -6.351096861727068)

    REGOLITH_CLEANUP_WAIT = 5.0 # seconds

    deliver_result = self._test_action(
      'Deliver',
      ow_lander.msg.DeliverAction,
      ow_lander.msg.DeliverGoal(),
      DELIVER_MAX_DURATION,
      DELIVER_EXPECTED_FINAL,
      self._assert_regolith_transports_and_delivers
    )

    # verify regolith is removed from the scene following deliver
    rospy.sleep(REGOLITH_CLEANUP_WAIT)
    self._assert_regolith_not_present(
      "Regolith models were not cleaned up %0.1fs after deliver"
        % REGOLITH_CLEANUP_WAIT
    )

if __name__ == '__main__':
  import rostest
  rostest.rosrun(PKG, 'terrain_interaction', TerrainInteraction)
