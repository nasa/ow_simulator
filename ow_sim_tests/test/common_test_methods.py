import rospy
import actionlib

import ow_lander.msg

def assertNothing():
  pass

"""
Returns true if an action is done
@param action_client: ROS action client object
"""
def is_action_done(action_client):
  return action_client.simple_state == actionlib.SimpleGoalState.DONE

"""
Calls an action asynchronously allowing checks to occur during its execution.
@param action_name: Name of action to be called
@param action: ROS action object
@param goal: Goal object that kwargs populates
@param max_duration: Max time allotted to action in seconds
@param condition_check: A function repeatedly called during action
       execution that asserts requirements for a successful action operation
@param condition_check_interval: The interval in seconds between calls to
       condition_check (default: 0.2)
@param expected_final: Final arm position to be compared with action result
       (default: None)
@param server_timeout: Time the action server is waited for (default: 10.0)
@param expected_final_tolerance: allowed distance between result.final and
       expected_final (default: 0.02)
@kwargs: Any properties of goal
"""
def test_action(test_object, action_name, action, goal, max_duration,
                condition_check = assertNothing,
                condition_check_interval = 0.2,
                expected_final = None,
                server_timeout = 10.0,
                expected_final_tolerance = 0.02,
                **kwargs):

  client = actionlib.SimpleActionClient(action_name, action)
  connected = client.wait_for_server(timeout=rospy.Duration(server_timeout))
  test_object.assertTrue(
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
      test_object.fail(
        "%s action does not contain parameter %s" % (action_name, key)
      )

  client.send_goal(goal)

  rospy.loginfo(
    "\n===%s action goal sent===%s" % (action_name, parameter_list)
  )

  # monitor for failed conditions during action execution
  start = rospy.get_time()
  elapsed = 0.0
  while not is_action_done(client) and elapsed < max_duration:
    condition_check()
    rospy.sleep(condition_check_interval)
    elapsed = rospy.get_time() - start

  test_object.assertLess(
    elapsed, max_duration,
    "Timeout reached waiting for %s action to finish!" % action_name
  )

  result = client.get_result()

  rospy.loginfo(
    "final position = (%0.3f, %0.3f, %0.3f)"
    % (result.final.x, result.final.y, result.final.z)
  )

  rospy.loginfo(
    "\n===%s action completed in %0.3fs===" % (action_name, elapsed)
  )

  # verify action ended where we expected
  if expected_final is not None:
    test_object._assert_point_is_near(
      result.final, expected_final, expected_final_tolerance,
      "Arm did not complete the %s action in the position expected!"
        % action_name
    )

  return result
