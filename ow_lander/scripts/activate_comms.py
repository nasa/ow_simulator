#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

from ow_lander import actions
from ow_lander import node_helper

import argparse

parser = argparse.ArgumentParser(
  formatter_class=argparse.ArgumentDefaultsHelpFormatter,
  description="Simulate the power load of the antenna uplinking/downlinking "
              "data. Communications are not modeled beyond their power usage."
)
parser.add_argument('--duration', '-d', type=float, default=10.0,
  help="The duration in seconds the uplink/downlink will take.")
args = parser.parse_args()

node_helper.call_single_use_action_client(
  actions.ActivateCommsServer, duration=args.duration
)
