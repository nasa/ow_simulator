#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import argparse

import rospy
import actionlib

import ow_lander.msg
from guarded_move_action_client import print_arguments

def LightSetIntensity_client():
  parser = argparse.ArgumentParser(
    description="Set the intensity of each lander mast spotlight " \
                "individually.",
    formatter_class=argparse.ArgumentDefaultsHelpFormatter
  )
  parser.add_argument('name', type=str, nargs='?', default='left',
    choices=['left', 'right'],
    help="Light identifier."
  )
  parser.add_argument('intensity', type=float, nargs='?', default=1.0,
    help="Intensity to set the light at. In the range [0.0, 1.0]. " \
         "0 is entirely off, and 1 is max intensity."
  )
  args = parser.parse_args()
  print_arguments(args)

  client = actionlib.SimpleActionClient(
    'LightSetIntensity', ow_lander.msg.LightSetIntensityAction
  )

  client.wait_for_server()

  goal = ow_lander.msg.LightSetIntensityGoal(
    name = args.name,
    intensity = args.intensity
  )

  # Sends the goal to the action server.
  client.send_goal(goal)

  # Waits for the server to finish performing the action.
  client.wait_for_result()

  return client.get_result()

if __name__ == '__main__':
  try:
    rospy.init_node('light_set_intensity_action_client_py')
    result = LightSetIntensity_client()
    rospy.loginfo("Result: %s", result)
  except rospy.ROSInterruptException:
    rospy.logerror("program interrupted before completion")
