# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import actionlib

from abc import ABC, abstractmethod

class ActionServerBase(ABC):
  def __init__(self, name, action, goal, feedback, result):
    self.name    = name
    self.action  = action
    self.goal    = goal
    self.feedack = feedback
    self.result  = result
    self._server  = actionlib.SimpleActionServer(
      self.name,
      action,
      execute_cb = self._on_action_called,
      auto_start = False
    )
  @abstractmethod
  def _on_action_called(self, goal):
    pass
  @staticmethod
  def _format_log(prefix, statement=None):
    if statement is None:
      return prefix
    else:
      return f"{prefix} - {statement}"
  def start_server(self):
    self._server.start()
  def publish_feedback(self):
    self._server.publish_feedback(self.feedback)
    pass
  def set_succeeded(self, msg=None):
    rospy.loginfo(self._format_log(f"{self.name}: Succeded", msg))
    self._server.set_succeeded(self.result)
  def set_aborted(self, msg=None):
    rospy.loginfo(self._format_log(f"{self.name}: Aborted", msg))
    self._server.set_aborted(self.result)
  def set_preempted(self, msg=None):
    rospy.loginfo(self._format_log(f"{self.name}: Preempted", msg))
    self._server.set_preempted()
