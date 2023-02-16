# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

"""Defines helper functions for creating server and client nodes from a
particular ActionServerBase child class
"""

import rospy
import actionlib

def camel_to_snake_case(s):
  return ''.join(['_'+c.lower() if c.isupper() else c for c in s]).lstrip('_')

def call_single_use_action_client(action_server_type, **kwargs):
  """Creates an anonymous action client node so a single call to the action
  server can be sent.
  kwargs -- parameters of the action's goal
  """
  node_name = camel_to_snake_case(action_server_type.name) + '_client'
  rospy.init_node(node_name, anonymous=True)
  client = actionlib.SimpleActionClient(
    action_server_type.name, action_server_type.action_type
  )
  try:
    client.wait_for_server()
    goal = action_server_type.goal_type(**kwargs)
    client.send_goal_and_wait(goal)
  except rospy.ROSInterruptException:
    rospy.logerr("Program interrupted before completion")
