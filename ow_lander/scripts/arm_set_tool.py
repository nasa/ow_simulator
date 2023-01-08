#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

from ow_lander import actions
from ow_lander import node_helper

import argparse

parser = argparse.ArgumentParser(
  formatter_class=argparse.ArgumentDefaultsHelpFormatter,
  description="Change end effector to specified tool (UNSUPPORTED)."
)
parser.add_argument(
  'tool', type=int, nargs='?', default=0,
  help="Tool choice (0 = Scoop, 1 = Drill)"
)
args = parser.parse_args()

node_helper.call_single_use_action_client(
  actions.ArmSetToolServer, **vars(args)
)
