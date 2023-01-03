#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import argparse
from ow_lander import node_helper
from ow_lander import actions

parser = argparse.ArgumentParser(
  formatter_class=argparse.ArgumentDefaultsHelpFormatter,
  description="Set the intensity of each lander mast spotlight " \
              "individually."
)
parser.add_argument(
  'name', type=str, nargs='?', default='left', choices=['left', 'right'],
  help="Light identifier."
)
parser.add_argument(
  'intensity', type=float, nargs='?', default=1.0,
  help="Intensity to set the light at. In the range [0.0, 1.0]. " \
       "0 is entirely off, and 1 is max intensity."
)
args = parser.parse_args()

node_helper.call_single_use_action_client(actions.LightSetIntensityServer,
  **vars(args))
