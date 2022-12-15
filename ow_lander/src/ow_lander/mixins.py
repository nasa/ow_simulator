# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import sys
import rospy
import actionlib
import moveit_commander

from ow_lander.arm_interface import ArmInterface
from ow_lander.trajectory_planner import ArmTrajectoryPlanner
from ow_lander.subscribers import LinkPositionSubscriber

from abc import ABC, abstractmethod

class ArmActionMixin:
  """Enables an action server to control the OceanWATERS arm. This or one of its
  children classes must be placed first in the inheritance statement.
  e.g.
  class FooArmActionServer(ArmActionMixin, ActionServerBase):
    ...
  """
  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)
    # initialize moveit interface for arm control
    moveit_commander.roscpp_initialize(sys.argv)
    # initialize/reference trajectory planner singleton (for external use)
    self._planner = ArmTrajectoryPlanner()
    # initialize/reference
    self._arm = ArmInterface()
    # initialize interface for querying scoop tip position
    self._arm_tip_monitor = LinkPositionSubscriber('lander::l_scoop_tip')
    self._start_server()

  def publish_action_feedback(self):
    """Publishes the action's feedback. Most arm actions publish the scoop tip
    position under the name "current" in their feedback, so that is what this
    method does by default, but it can be overridden by the child class.
    """
    self._publish_feedback(current=self._arm_tip_monitor.get_link_position())

class ArmToGroupStateMixin(ArmActionMixin, ABC):
  """A specialization of an arm action mixin that moves the arm directly to
  a group state, which is a series of joint angles."""

  def __init__(self, *args, **kwargs):
    super().__init__(*args, **kwargs)

  """The named group state which the arm will move directly to when this
  particular type of action is executed. Must be set by child class!
  """
  @property
  @abstractmethod
  def target_group_state(self):
    pass

  def execute_action(self, _goal):
    try:
      self._arm.checkout_arm(self.name)
      plan = self._planner.plan_arm_to_target(self.target_group_state)
      self._arm.execute_arm_trajectory(plan)
    except RuntimeError as err:
      self._set_aborted(str(err),
        final=self._arm_tip_monitor.get_link_position())
    else:
      self._set_succeeded("Arm trajectory succeeded",
        final=self._arm_tip_monitor.get_link_position())
    finally:
      self._arm.checkin_arm(self.name)
