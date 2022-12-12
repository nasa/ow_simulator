#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

from ow_lander.actions import lander
from ow_lander import node_helper

node_helper.spin_action_server(lander.CameraCaptureServer)
