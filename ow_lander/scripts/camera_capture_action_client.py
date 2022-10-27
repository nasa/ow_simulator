#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import actionlib
import argparse
import ow_lander.msg
from guarded_move_action_client import print_arguments


def camera_capture_client():
  parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument('exposure', type=float,
                      help='Camera exposure in seconds. Must be > 0 or current exposure will be used.',
                      nargs='?', default=-1)
  args = parser.parse_args()
  print_arguments(args)

  client = actionlib.SimpleActionClient('CameraCapture', ow_lander.msg.CameraCaptureAction)
  client.wait_for_server()

  goal = ow_lander.msg.CameraCaptureGoal(exposure=args.exposure)

  # Sends the goal to the action server.
  client.send_goal(goal)

  # Waits for the server to finish performing the action.
  client.wait_for_result()

  # Prints out the result of executing the action
  return client.get_result()


if __name__ == '__main__':
  try:
    # Initializes a rospy node
    rospy.init_node('camera_capture_client_py')
    result = camera_capture_client()
    rospy.loginfo(f'Result: {result}')
  except rospy.ROSInterruptException:
    rospy.logerror('program interrupted before completion')

