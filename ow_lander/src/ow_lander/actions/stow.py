# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy

import ow_lander.msg

from ow_lander.server import ActionServerBase
from ow_lander.arm_action_mixin import ArmActionMixin

class StowServer(ArmActionMixin, ActionServerBase):

  # UNIFICATION TODO: rename "Stow" to "ArmStow"
  name          = 'Stow'
  action_type   = ow_lander.msg.StowAction
  goal_type     = ow_lander.msg.StowGoal
  feedback_type = ow_lander.msg.StowFeedback
  result_type   = ow_lander.msg.StowResult

  def __init__(self):
    super().__init__()
    self._start_server()

  def execute_action(self, _goal):
    try:
      ArmActionMixin._checkout_arm()
      plan = self._planner.plan_arm_to_target('arm_stowed')
      self._execute_arm_trajectory(plan)
    except RuntimeError as err:
      self._set_aborted(str(err), final=self._get_arm_tip_position())
    else:
      self._set_succeeded("Arm trajectory succeeded",
        final=self._get_arm_tip_position())
    finally:
      ArmActionMixin._checkin_arm()
