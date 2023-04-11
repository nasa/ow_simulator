# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import actionlib

from abc import ABC, abstractmethod

class ActionServerBase(ABC):
  """A base class that standardizes handling of action goals, results, feedback,
  and logging for all OceanWATERS actions.
  """

  def __init__(self):
    self._server  = actionlib.SimpleActionServer(
      self.name,
      self.action_type,
      execute_cb = self.__on_action_called,
      auto_start = False
    )

  """The string the action server is registered under. Must be overridden!"""
  @property
  @abstractmethod
  def name(self):
    pass

  """The following *_type properties are auto-generated from the *.action file
  and made for import in the corresponding package. All must be overridden!
  """
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
    """Called whenever the action is called. Must be overridden!"""
    pass

  def _start_server(self):
    """Child class calls this after it has initialized its data members."""
    self._server.start()

  def _is_preempt_requested(self):
    """Check if a preempt has been requested."""
    return self._server.is_preempt_requested()

  def _publish_feedback(self, **kwargs):
    """Publish action feedback during execution of the action. This is not
    required if action has an empty feedback type.
    kwargs -- Keyword arguments that match fields in feedback_type.
    """
    try:
      feedback = self.feedback_type(**kwargs)
    except AttributeError as err:
      rospy.logerr_once(err)
      return
    self._server.publish_feedback(feedback)

  def _set_succeeded(self, msg, **kwargs):
    """Declare action succeeded, and publish its result.
    msg    -- Message that explains why success occurred. It is logged to ROS
              info and passed to action clients.
    kwargs -- Keyword arguments that match fields in result_type. Leave empty
              if result type is empty.
    """
    rospy.loginfo(self.__format_result_msg(f"{self.name}: Succeeded", msg))
    result, msg = self.__create_result(msg, **kwargs)
    self._server.set_succeeded(result, msg)

  def _set_preempted(self, msg, **kwargs):
    """Declare action was preempted, and publish its results.
    msg    -- Message that explains why action was preempted. It is logged to
              ROS info and passed to action clients.
    kwargs -- Keyword arguments that match fields in result_type. Leave empty
              if result type is empty or no results were produced before
              preempt.
    """
    rospy.loginfo(self.__format_result_msg(f"{self.name}: Preempted", msg))
    result, msg = self.__create_result(msg, **kwargs)
    self._server.set_preempted(result, msg)

  def _set_aborted(self, msg, **kwargs):
    """Declare action was aborted, and publish its results.
    msg    -- Message that explains why action was aborted. It is logged to
              ROS error and passed to action clients.
    kwargs -- Keyword arguments that match fields in result_type. Leave empty
              if result type is empty or no results were produced before abort.
    """
    rospy.logerr(self.__format_result_msg(f"{self.name}: Aborted", msg))
    result, msg = self.__create_result(msg, **kwargs)
    self._server.set_aborted(result, msg)

  def __on_action_called(self, goal):
    if not isinstance(goal, self.goal_type):
      rospy.logerr("Action server passed an unexpected action goal type." \
                   "This should never happen!")
      return
    rospy.loginfo(f"{self.name} action started")
    self.execute_action(goal)
    rospy.loginfo(f"{self.name} action complete")

  def __create_result(self, msg, **kwargs):
    result = self.result_type()
    try:
      result = self.result_type(**kwargs)
    except AttributeError as err:
      attribute_err = f"{err}; an empty action result will be published"
      rospy.logerr(attribute_err)
      # append the error to pre-existing message so action clients have all the
      # information about what happened
      msg += "\n" + attribute_err
    return result, msg

  @staticmethod
  def __format_result_msg(prefix, msg=""):
    if msg == "":
      return prefix
    else:
      return f"{prefix} - {msg}"
