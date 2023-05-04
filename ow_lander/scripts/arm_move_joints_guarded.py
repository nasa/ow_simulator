#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

from ow_lander import actions
from ow_lander import node_helper

import argparse

parser = argparse.ArgumentParser(
  formatter_class=argparse.ArgumentDefaultsHelpFormatter,
  description="Move all arm joints to either absolute or relative positions. "
              "Movement will cease if the distal pitch force/torque sensor "
              "detects a force/torque larger than the provided values. Torque "
              "of individual arm joints will not cause movement to stop.")
parser.add_argument('shoulder_yaw', type=float,
  help="Shoulder yaw joint position/rotation in radians")
parser.add_argument('shoulder_pitch', type=float,
  help="Shoulder pitch joint position/rotation in radians")
parser.add_argument('proximal_pitch', type=float,
  help="Proximal pitch joint position/rotation in radians")
parser.add_argument('distal_pitch', type=float,
  help="Distal pitch joint position/rotation in radians")
parser.add_argument('hand_yaw', type=float,
  help="Hand yaw joint position/rotation in radians")
parser.add_argument('scoop_yaw', type=float,
  help="Scoop yaw joint position/rotation in radians")
parser.add_argument('--relative', '-r', action='store_true', default=False,
  help="Joints will move to the provided positions relative to their current "
       "positions.")
parser.add_argument('--force', type=float, default=200,
  help="Arm will stop when F/T sensor encounters a force above this value in "
       "Newtons")
parser.add_argument('--torque', type=float, default=100,
  help="Arm will stop when F/T sensor encounters a torque above this value "
       "in Newton*meters")
args = parser.parse_args()

angles_arg = [
  args.shoulder_yaw,
  args.shoulder_pitch,
  args.proximal_pitch,
  args.distal_pitch,
  args.hand_yaw,
  args.scoop_yaw
]

node_helper.call_single_use_action_client(actions.ArmMoveJointsGuardedServer,
  relative=args.relative, angles=angles_arg,
  force_threshold=args.force, torque_threshold=args.torque)
