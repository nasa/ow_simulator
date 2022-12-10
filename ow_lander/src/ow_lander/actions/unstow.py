# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy

import ow_lander.msg

from ow_lander.server import ActionServerBase
from ow_lander.arm_action_mixin import ArmToGroupStateMixin

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
