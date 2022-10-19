import math

import rospy
import actionlib

from geometry_msgs.msg import Point
import ow_lander.msg

"""
Computes the 3D distance between two geometry_msgs.msg Points
"""
def distance(p1, p2):
  v = Point(p2.x - p1.x,
            p2.y - p1.y,
            p2.z - p1.z)
  return math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z)

def assert_nothing():
  pass

"""
Asserts two Points are near each
@param  p1: First point
@param  p2: Second point
@param  delta: Distance under which points are considered "near"
@param  msg: Assert message
"""
def assert_point_is_near(test_object, p1, p2, delta, msg):
  test_object.assertLessEqual(distance(p1, p2), delta, msg)

"""
Asserts two Points are far enough from each
@param  p1: First point
@param  p2: Second point
@param  delta: Distance above which points are considered "far"
@param  msg: Assert message
"""
def assert_point_is_far(test_object, p1, p2, delta, msg):
  test_object.assertGreater(distance(p1, p2), delta, msg)


"""
Returns true if an action is done
@param  action_client: ROS action client object
"""
def is_action_done(action_client):
  return action_client.simple_state == actionlib.SimpleGoalState.DONE

"""
Calls an action asynchronously allowing checks to occur during its execution.
@param  test_object: Object of type unittest.TestCase
@param  action_name: Name of action to be called
@param  action: ROS action object
@param  goal: Goal object that kwargs populates
@param  max_duration: Max time allotted to action in seconds.
@param  condition_check: A function repeatedly called during action
        execution that asserts requirements for a successful action operation
@param  condition_check_interval: The interval in seconds between calls to
        condition_check (default: 0.2)
@param  server_timeout: Time the action server is waited for (default: 10.0)
"""
def test_action_noyaml(test_object, action_name, action, goal, max_duration,
                       condition_check = assert_nothing,
                       condition_check_interval = 0.2,
                       server_timeout = 10.0):

  rospy.loginfo("max_duration = %f" % max_duration)

  client = actionlib.SimpleActionClient(action_name, action)
  connected = client.wait_for_server(timeout=rospy.Duration(server_timeout))
  test_object.assertTrue(
    connected,
    "Timeout exceeded waiting for %s action server" % action_name
  )

  client.send_goal(goal)

  rospy.loginfo(
    "\n===%s action goal sent===\n%s" % (action_name, goal)
  )

  # monitor for failed conditions during action execution
  start = rospy.get_time()
  elapsed = 0.0
  while not is_action_done(client):
    condition_check()
    rospy.sleep(condition_check_interval)
    elapsed = rospy.get_time() - start
    if elapsed > max_duration:
      break

  test_object.assertLess(
    elapsed, max_duration,
    "Timeout reached waiting for %s action to finish!" % action_name
  )

  rospy.loginfo(
    "\n=== %s action completed in %0.3fs ===" % (action_name, elapsed)
  )

  return client.get_result()

"""
Calls an action asynchronously allowing checks to occur during its execution.
See test_action for explanation of other parameters.
@param  test_object: Object of type unittest.TestCase
@param  action_name: Name of action to be called
@param  action: ROS action object
@param  goal: Goal object that kwargs populates
@param  max_duration: Max time allotted to action in seconds.
@param  expected_final: Final arm position to be compared with action result
        (default: None)
@param  expected_final_tolerance: allowed distance between result.final and
        expected_final (default: 0.02)
@kwargs keyword arguments of test_arm_action_noyaml
"""
def test_arm_action_noyaml(test_object, action_name, action, goal, max_duration,
                           expected_final = None,
                           expected_final_tolerance = 0.02,
                           **kwargs):
  result = test_action_noyaml(
    test_object, action_name, action, goal, max_duration, **kwargs
  )

  rospy.loginfo(
    "=== arm final position is (%0.3f, %0.3f, %0.3f) ==="
    % (result.final.x, result.final.y, result.final.z)
  )

  # verify action ended where we expected
  if expected_final:
    assert_point_is_near(test_object,
      result.final, expected_final, expected_final_tolerance,
      "Arm did not complete the %s action in the position expected!"
        % action_name
    )

  return result

def get_test_results_uri(test_name, test_unit, parameter = None):
  if parameter:
    return '/%s/arm_action_results/%s/%s' % (test_name, test_unit, parameter)
  else:
    return '/%s/arm_action_results/%s' % (test_name, test_unit)

def is_test_results_present(test_name, test_unit):
  return rospy.has_param(get_test_results_uri(test_name, test_unit))

def get_param_duration(test_name, test_unit):
  DEFAULT = 9999
  return rospy.get_param(
    get_test_results_uri(test_name, test_unit, 'duration'), DEFAULT
  )

def get_param_final(test_name, test_unit):
  DEFAULT = None
  uri = get_test_results_uri(test_name, test_unit, 'final')
  if not rospy.has_param(uri):
    return DEFAULT
  final = rospy.get_param(uri)
  return Point(final['x'], final['y'], final['z'])

def get_param_final_tolerance(test_name, test_unit):
  DEFAULT = 0.02
  return rospy.get_param(
    get_test_results_uri(test_name, test_unit, 'final_tolerance'), DEFAULT
  )

"""
Version of test_action_noyaml that grabs test result parameters from a
configuration file.
@param  test_object: Object of type unittest.TestCase
@param  action_name: Name of action to be called
@param  action: ROS action object
@param  goal: Goal object that kwargs populates
@param  test_name: Name of rostest. Used to lookup results from a config file.
@param  unit_name: Name of test method that calls this function. Can be
        typically be filled with __name__.
@kwargs keyword arguments of test_action_noyaml
"""
def test_action(test_object, action_name, action, goal,
                test_name, unit_name, **kwargs):
  if not is_test_results_present(test_name, unit_name):
    rospy.logerror("Test results not found.")
  test_action_noyaml(
    test_object, action_name, action, goal,
    get_param_duration(test_name, unit_name),
    **kwargs
  )

"""
Version of test_arm_action_noyaml that grabs test result parameters from a
configuration file.
@param  test_object: Object of type unittest.TestCase
@param  action_name: Name of action to be called
@param  action: ROS action object
@param  goal: Goal object that kwargs populates
@param  test_name: Name of rostest. Used to lookup results from a config file.
@param  unit_name: Name of test method that calls this function. Can be
        typically be filled with __name__.
@kwargs keyword arguments of test_arm_action_noyaml
"""
def test_arm_action(test_object, action_name, action, goal,
                    test_name, unit_name, **kwargs):
  if not is_test_results_present(test_name, unit_name):
    rospy.logerror("Test results not found.")
  test_arm_action_noyaml(
    test_object, action_name, action, goal,
    get_param_duration(test_name, unit_name),
    expected_final = get_param_final(test_name, unit_name),
    expected_final_tolerance = get_param_final_tolerance(test_name, unit_name),
    **kwargs
  )
