# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy

import ow_lander.msg

from ow_lander.server import ActionServerBase
from ow_lander.arm_action_mixin import ArmActionMixin, ArmToGroupStateMixin

class UnstowServer(ArmToGroupStateMixin, ActionServerBase):

  # UNIFICATION TODO: rename "Stow" to "ArmStow"
  name          = 'Unstow'
  action_type   = ow_lander.msg.UnstowAction
  goal_type     = ow_lander.msg.UnstowGoal
  feedback_type = ow_lander.msg.UnstowFeedback
  result_type   = ow_lander.msg.UnstowResult

  target_group_state = 'arm_unstowed'

  def __init__(self):
    super().__init__()

class StowServer(ArmToGroupStateMixin, ActionServerBase):

  # UNIFICATION TODO: rename "Stow" to "ArmStow"
  name          = 'Stow'
  action_type   = ow_lander.msg.StowAction
  goal_type     = ow_lander.msg.StowGoal
  feedback_type = ow_lander.msg.StowFeedback
  result_type   = ow_lander.msg.StowResult

  target_group_state = 'arm_stowed'

  def __init__(self):
    super().__init__()

class StopServer(ArmActionMixin, ActionServerBase):

  name          = 'Stop'
  action_type   = ow_lander.msg.StopAction
  goal_type     = ow_lander.msg.StopGoal
  feedback_type = ow_lander.msg.StopFeedback
  result_type   = ow_lander.msg.StopResult

  def __init__(self):
    super().__init__()

  def execute_action(self, _goal):
    if self._arm.stop_arm():
      self._set_succeeded("Arm trajectory stopped",
        final=self._arm_tip_monitor.get_link_position())
    else:
      self._set_aborted("No arm trajectory to stop",
        final=self._arm_tip_monitor.get_link_position())

class GrindServer(ArmActionMixin, ActionServerBase):

  name          = 'Grind'
  action_type   = ow_lander.msg.GrindAction
  goal_type     = ow_lander.msg.GrindGoal
  feedback_type = ow_lander.msg.GrindFeedback
  result_type   = ow_lander.msg.GrindResult

  def __init__(self):
    super().__init__()

  def execute_action(self, goal):
    try:
      self._arm.checkout_arm(self.name)
      self._arm.switch_to_grinder_controller()
      plan = self._planner.grind(goal)
      self._arm.execute_arm_trajectory(plan)
    except RuntimeError as err:
      self._set_aborted(str(err),
        final=self._arm_tip_monitor.get_link_position())
    else:
      self._set_succeeded("Arm trajectory succeeded",
        final=self._arm_tip_monitor.get_link_position())
    finally:
      self._arm.switch_to_arm_controller()
      self._arm.checkin_arm(self.name)
