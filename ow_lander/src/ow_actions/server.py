# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import actionlib

from abc import ABC, abstractmethod

class ActionServerBase(ABC):

  def __init__(self):
    self._server  = actionlib.SimpleActionServer(
      self.name,
      self.action_type,
      execute_cb = self._on_action_called,
      auto_start = False
    )

  @property
  @abstractmethod
  def name(self):
    pass
  @property
  @abstractmethod
  def action_type(self):
    pass
  @property
  @abstractmethod
  def goal_type(self):
    pass
  @property
  @abstractmethod
  def feedback_type(self):
    pass
  @property
  @abstractmethod
  def result_type(self):
    pass

  @abstractmethod
  def execute_action(self, goal):
    pass

  def _on_action_called(self, goal):
    if not isinstance(goal, self.goal_type):
      rospy.logfatal("Action server passed an unexpected action goal type.")
      return
    rospy.loginfo(f"{self.name} action started")
    self.execute_action(goal)

  def _start_server(self):
    self._server.start()

  def _publish_feedback(self, feedback):
    if not isinstance(feedback, self.feedback_type):
      rospy.logfatal("Action server passed an unexpected action feedback type.")
      return
    self._server.publish_feedback(feedback)

  def _set_succeeded(self, result, reason=None):
    if not isinstance(result, self.result_type):
      rospy.logfatal("Action server passed an unexepcted action result type.")
      return
    rospy.loginfo(self.__format_result_msg(f"{self.name}: Succeded", reason))
    self._server.set_succeeded(result)

  def _set_preempted(self, result, reason=None):
    if not isinstance(result, self.result_type):
      rospy.logfatal("Action server passed an unexepcted action result type.")
      return
    rospy.loginfo(self.__format_result_msg(f"{self.name}: Preempted", reason))
    self._server.set_preempted()

  def _set_aborted(self, result, reason=None):
    if not isinstance(result, self.result_type):
      rospy.logfatal("Action server passed an unexepcted action result type.")
      return
    rospy.logerr(self.__format_result_msg(f"{self.name}: Aborted", reason))
    self._server.set_aborted(result)

  @staticmethod
  def __format_result_msg(prefix, statement=None):
    if statement is None:
      return prefix
    else:
      return f"{prefix} - {statement}"
