# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import actionlib

from abc import ABC, abstractmethod

def _format_log(prefix, statement=None):
  if statement is None:
    return prefix
  else:
    return f"{prefix} - {statement}"

class OWActionServer(ABC):

  def __init__(self, name, action, goal, feedback, result):
    self.name    = name
    self.action  = action
    self.goal    = goal
    self.feedack = feedback
    self.result  = result
    self._server  = actionlib.SimpleActionServer(
      self._action_name,
      action,
      execute_cb = self._on_action_called,
      auto_start = False
    )

  def start_server(self):
    self._server.start()

  @abstractmethod
  def _on_action_called(self, goal):
    pass

  def publish_feedback(self):
    self._server.publish_feedback(self.feedback)
    pass

  def set_succeeded(self, msg=None):
    rospy.loginfo(_format_log(f"{self.name}: Succeded", msg))
    self._server.set_succeded(self.result)

  def set_aborted(self, msg=None):
    rospy.loginfo(_format_log(f"{self.name}: Aborted", msg))
    self._server.set_aborted(self.result)

  def set_preempted(self, msg=None):
    rospy.loginfo(_format_log(f"{self.name}: Preempted", msg))
    self._server.set_preempted()
