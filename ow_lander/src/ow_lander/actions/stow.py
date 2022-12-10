# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy

import ow_lander.msg

from ow_lander.server import ActionServerBase
from ow_lander.arm_action_mixin import ArmToGroupStateMixin

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
