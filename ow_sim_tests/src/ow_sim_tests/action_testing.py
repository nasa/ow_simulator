import math

import rospy
import actionlib

from geometry_msgs.msg import Point

ignore_action_checks = False

def distance(p1, p2):
  """Computes the 3D distance between two geometry_msgs.msg Points"""
  v = Point(p2.x - p1.x,
            p2.y - p1.y,
            p2.z - p1.z)
  return math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)

  def is_action_done(action_client):
    """Returns true if an action is done
    @param  action_client: ROS action client object
    """
    return action_client.simple_state == actionlib.SimpleGoalState.DONE


class ActionTesterMixin:

  ignore_acttion_checks = False

  @classmethod
  def set_ignore_action_checks(cls, value : bool):
    """Set the value of the ignore_action_checks. When true, action unit
    testsare allowed to exceed their maximum duration without producing a
    failure (and may execute indefinitely). For arm action tests the final
    position of the arm will additionally never produce a failure.
    value -- New value of the boolean ignore_action_checks
    """
    cls.ignore_action_checks = value

  def assert_nothing(self):
    """Does nothing. Simply here to be act as a non-check function for the
    test_action and test_arm_action functions.
    """
    pass


  def assert_in_closed_range(self, value, low, high, msg=None):
    """Asserts a value is between low and high (inclusive)
    value -- Value being tested
    low   -- Lowest value it must be greater than or equal to
    high  -- Highest value it must be less than or equal to
    msg   -- Assert message (default: None)
    """
    self.assertGreaterEqual(value, low, msg)
    self.assertLessEqual(value, high, msg)

  def assert_point_is_within_deviation(self, p, expected, dev, msg=None):
    """Asserts two Points are withing a deviation vector of each other
    p        -- Point being tested
    expected -- Expected Point
    dev      -- A vector of acceptable deviations in x, y, and z
    msg      -- Assert message (default: None)
    """
    DEFAULT_MSG = "%s-coordinate is out of acceptable range"
    self.assert_in_closed_range(
      p.x, expected.x - dev.x, expected.x + dev.x,
      DEFAULT_MSG % 'x' if msg == None else msg
    )
    self.assert_in_closed_range(
      p.y, expected.y - dev.y, expected.y + dev.y,
      DEFAULT_MSG % 'y' if msg == None else msg
    )
    self.assert_in_closed_range(
      p.z, expected.z - dev.z, expected.z + dev.z,
      DEFAULT_MSG % 'z' if msg == None else msg
    )

  """
  Calls an action asynchronously allowing checks to occur during its execution.
  test_object  -- Object of type unittest.TestCase
  action_name  -- Name of action to be called
  action       -- ROS action object
  goal         -- Goal object passed to action server
  max_duration -- Max time allotted to action in seconds.
  expected_end_state -- An integer that maps to values in actionlib.GoalStatus.
    It provides the expected state final state of the action call. This allows
    testing of action calls that are expected to fail.
    (default: actionlib.GoalStatus.SUCCEEDED or 3)
  condition_check -- A function repeatedly called during action execution that
    asserts requirements for a successful action operation.
  condition_check_interval -- The interval in seconds between calls to
    condition_check (default: 0.2)
  server_timeout -- Time the action server is waited for (default: 10.0)
  """
  def test_action_noyaml(test_object, action_name, action, goal, max_duration,
                         condition_check = assert_nothing,
                         condition_check_interval = 0.2,
                         server_timeout = 10.0,
                         expected_end_state = actionlib.GoalStatus.SUCCEEDED):
    client = actionlib.SimpleActionClient(action_name, action)
    connected = client.wait_for_server(timeout=rospy.Duration(server_timeout))
    test_object.assertTrue(
      connected,
      "Timeout exceeded waiting for %s action server" % action_name
    )
    client.send_goal(goal)
    # monitor for failed conditions during action execution
    start = rospy.get_time()
    condition_failure = None
    while not is_action_done(client):
      rospy.sleep(condition_check_interval)
      # catch the first failure only
      if condition_failure is None:
        try:
          condition_check()
        except AssertionError as failure:
          condition_failure = failure

    elapsed = rospy.get_time() - start
    # only fail the test when the action has completed or timed out
    if not condition_failure is None:
      raise condition_failure
    test_object.assertLess(
      elapsed, max_duration,
      "Action %s took longer than expected to finish!" % action_name
    )
    end_state = client.get_state()
    test_object.assertEqual(
      end_state, expected_end_state,
      f"Action {action_name} completed with state "
      f"{actionlib.GoalStatus.to_string(end_state)} instead of the expected end "
      f"state of {actionlib.GoalStatus.to_string(expected_end_state)}."
    )
    ## TOOD: return client instead so caller can access all information about
    ##       the action results
    return client.get_result(), elapsed

  """
  Calls an action asynchronously allowing checks to occur during its execution.
  @param  test_object: Object of type unittest.TestCase
  @param  action_name: Name of action to be called
  @param  action: ROS action object
  @param  goal: Goal object passed to action server
  @param  max_duration: Max time allotted to action in seconds.
  @param  expected_end_state: An integer that maps to values in
          actionlib.GoalStatus. It provides the expected state final state of the
          action call. This allows testing of action calls that are expected to
          fail. (default: actionlib.GoalStatus.SUCCEEDED or 3)
  @param  expected_final: Final arm position to be compared with action result
          (default: None)
  @param  expected_final_tolerance: allowed distance between result.final and
          expected_final (default: 0.02)
  @kwargs keyword arguments of test_arm_action_noyaml
  """
  def test_arm_action_noyaml(test_object, action_name, action, goal, max_duration,
                             expected_final = None,
                             expected_final_tolerance = Point(0.2, 0.2, 0.2),
                             **kwargs):
    result, elapsed = test_action_noyaml(
      test_object, action_name, action, goal, max_duration, **kwargs
    )
      # verify action ended where we expected
    if expected_final and hasattr(result, 'final'):
      assert_point_is_within_deviation(test_object,
        result.final, expected_final, expected_final_tolerance
      )
    return result, elapsed

  def print_action_start(unit_name, goal):
    rospy.loginfo("\n=== %s goal sent ===\n%s", unit_name, goal)

  def print_action_complete(unit_name, elapsed):
    rospy.loginfo("\n=== %s completed in %0.4fs ===", unit_name, elapsed)

  def print_arm_action_final(unit_name, final):
    rospy.loginfo(
      "\n=== %s completed with arm in position (%0.4f, %0.4f, %0.4f) ===",
      unit_name, final.x, final.y, final.z
    )

  def are_test_parameters_available(test_name, unit_name):
    return rospy.has_param(
      '/%s/test_action_statistics/%s' % (test_name, unit_name)
    )

  def get_test_parameter_mean(test_name, unit_name, parameter):
    return rospy.get_param(
      '/%s/test_action_statistics/%s/%s/mean' % (test_name, unit_name, parameter)
    )

  def get_test_parameter_std(test_name, unit_name, parameter):
    return rospy.get_param(
      '/%s/test_action_statistics/%s/%s/std' % (test_name, unit_name, parameter)
    )

  def get_max_duration(test_name, unit_name, std_factor, min_tolerance):
    duration = get_test_parameter_mean(test_name, unit_name, 'duration')
    duration_std = get_test_parameter_std(test_name, unit_name, 'duration')
    return duration + max(std_factor * duration_std, min_tolerance)

  """
  Version of test_action_noyaml that grabs testing parameters from a
  configuration file, typically generated by the arm_test_analysis.py script.
  @param  test_object: Object of type unittest.TestCase
  @param  action_name: Name of action to be called
  @param  action: ROS action object
  @param  goal: Goal object passed to action server
  @param  test_name: Name of rostest. Used to lookup results from a config file.
  @param  unit_name: Name of unit test that calls this function. Used to lookup
          results from config file.
  @param  duration_std_factor: The product of this and the unit test's duration
          standard deviation are added to the duration mean to yield a maximum
          allowed duration for the test. In statistical nomenclature, assigning a
          factor 2.0 is the same as allowing the test to go on for 2-sigma longer
          than it's mean duration. (default: 4)
  @param  duration_minimum_tolerance: Minimum tolerance for the unit duration. The
          duration tolerance is the value added to the mean duration to yield a
          maximum duration. (default: 10)
  @kwargs keyword arguments of test_action_noyaml
  """
  def test_action(test_object, action_name, action, goal,
                  test_name, unit_name,
                  duration_std_factor = 4,
                  duration_minimum_tolerance = 10, # seconds
                  **kwargs):
    print_action_start(unit_name, goal)
    result, elapsed = None, None
    if ignore_action_checks:
      result, elapsed = test_action_noyaml(
        test_object, action_name, action, goal, 99999999.0,
        **kwargs
      )
    else:
      if not are_test_parameters_available(test_name, unit_name):
        rospy.logerr("Test results not found.")
      max_duration = get_max_duration(
        test_name, unit_name, duration_std_factor, duration_minimum_tolerance
      )
      result, elapsed = test_action_noyaml(
        test_object, action_name, action, goal,
        max_duration,
        **kwargs
      )
    print_action_complete(unit_name, elapsed)

  """
  Version of test_arm_action_noyaml that grabs testing parameters from a
  configuration file, typically generated by the arm_test_analysis.py script.
  @param  test_object: Object of type unittest.TestCase
  @param  action_name: Name of action to be called
  @param  action: ROS action object
  @param  goal: Goal object passed to action server.
  @param  test_name: Name of rostest. Used to lookup results from a config file.
  @param  unit_name: Name of unit test that calls this function. Used to lookup
          results from config file.
  @param  duration_std_factor: The product of this and the unit test's duration
          standard deviation are added to the duration mean to yield a maximum
          allowed duration for the test. In statistical nomenclature, assigning a
          factor 2.0 is the same as allowing the test to go on for 2-sigma longer
          than it's mean duration. (default: 4)
  @param  duration_minimum_tolerance: Minimum tolerance for the unit duration. The
          duration tolerance is the value added to the mean duration to yield a
          maximum duration. (default: 10)
  @param  expected_final_std_factor: The product of this and the unit test's final
          arm position standard deviation vector yield a tolerance around the
          mean final arm position of acceptable positions the arm should finalize
          its trajectory in. In statistical nomenclature, assigning a factor of
          2.0 is the same as allowing the arm's final position to be within
          2-sigma of its mean final position. The standard deviation vector
          provides a standard deviation for each of the three coordinates
          (x, y, and z), so the arm's final position is verified to be within an
          axis-aligned box of dimensions defined by the scalar-vector product of
          this value and the standard deviation vector. (default: 4)
  @param  expected_final_minimum_tolerance: Minimum tolerance used for the arm's
          final position in any of the three coordinates (x, y, and z).
          (default: 0.01)
  @kwargs keyword arguments of test_arm_action_noyaml
  """
  def test_arm_action(test_object, action_name, action, goal,
                      test_name, unit_name,
                      duration_std_factor = 4,
                      duration_minimum_tolerance = 10, # seconds
                      expected_final_std_factor = 4,
                      expected_final_minimum_tolerance = 0.01, # meters
                      **kwargs):
    print_action_start(unit_name, goal)
    result, elapsed = None, None
    if ignore_action_checks:
      result, elapsed = test_arm_action_noyaml(
        test_object, action_name, action, goal, 99999999.0,
        **kwargs
      )
    else:
      if not are_test_parameters_available(test_name, unit_name):
        rospy.logerr("Test results not found.")
      max_duration = get_max_duration(
        test_name, unit_name, duration_std_factor, duration_minimum_tolerance
      )
      final = Point(
        get_test_parameter_mean(test_name, unit_name, 'final_x'),
        get_test_parameter_mean(test_name, unit_name, 'final_y'),
        get_test_parameter_mean(test_name, unit_name, 'final_z')
      )
      final_tolerance = Point(
        max(
          expected_final_std_factor * get_test_parameter_std(test_name, unit_name, 'final_x'),
          expected_final_minimum_tolerance
        ),
        max(
          expected_final_std_factor * get_test_parameter_std(test_name, unit_name, 'final_y'),
          expected_final_minimum_tolerance
        ),
        max(
          expected_final_std_factor * get_test_parameter_std(test_name, unit_name, 'final_z'),
          expected_final_minimum_tolerance
        )
      )
      result, elapsed = test_arm_action_noyaml(
        test_object, action_name, action, goal,
        max_duration,
        expected_final = final,
        expected_final_tolerance = final_tolerance,
        **kwargs
      )
    if hasattr(result, 'final'):
      print_arm_action_final(unit_name, result.final)
    print_action_complete(unit_name, elapsed)
