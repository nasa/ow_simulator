# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import actionlib

class ActionClient(object):
  def __init__(self, name, action):
    self.client = actionlib.SimpleActionClient(name, action)
    self.client.wait_for_server()
  def call(self, goal):
    self.client.send_goal_and_wait(goal)
    return self.client.get_result()
