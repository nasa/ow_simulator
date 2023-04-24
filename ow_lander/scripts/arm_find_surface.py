#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

from ow_lander import actions
from ow_lander import node_helper
from ow_lander import constants

import argparse

from geometry_msgs.msg import Point, Vector3

parser = argparse.ArgumentParser(
  formatter_class=argparse.ArgumentDefaultsHelpFormatter,
  description="Determine the position of a surface by moving the arm's "
              "end-effector towards surface until sufficient resistance is "
              "encountered.")
parser.add_argument('--frame', '-f', type=int, default=0,
  choices=constants.FRAME_ID_MAP.keys(),
  help="The frame index corresponding to the frame that orientation and "
       "position are relative to. " + str(constants.FRAME_ID_MAP))
parser.add_argument('--relative', '-r', action='store_true', default=False,
  help="Position will be interpreted as relative to the current position")
parser.add_argument('-x', type=float, default=0,
  help="X-coordinate or the change in the X-coordinate")
parser.add_argument('-y', type=float, default=0,
  help="Y-coordinate or the change in the Y-coordinate")
parser.add_argument('-z', type=float, default=0,
  help="Z-coordinate or the change in the Z-coordinate")
parser.add_argument('--normal', '-n', type=float, nargs=3,
  default=[0, 0, -1], metavar=('Nx', 'Ny', 'Nz'),
  help="Normal vector in which arm will move a distance + overdrive in until "
       "either the force or torque thresholds are breached. This will always "
       "be interpreted in the base_link frame.")
parser.add_argument('--distance', '-d', type=float, default=0.2,
  help="Distance away from the estimated surface position that the "
       "end-effector starts its trajectory.")
parser.add_argument('--overdrive', '-o', type=float, default=0.05,
  help="Distance beyond the estimated surface position through which the "
       "end-effector will keep pushing.")
parser.add_argument('--force', type=float, default=200,
  help="Arm will stop when F/T sensor encounters a force above this value in "
       "Newtons")
parser.add_argument('--torque', type=float, default=100,
  help="Arm will stop when F/T sensor encounters a torque above this value "
       "in Newton*meters")

args = parser.parse_args()

position_arg = Point(args.x, args.y, args.z)
normal_arg = Vector3(args.normal[0], args.normal[1], args.normal[2])

node_helper.call_single_use_action_client(actions.ArmFindSurfaceServer,
  frame=args.frame, relative=args.relative, position=position_arg,
  normal=normal_arg, distance=args.distance, overdrive=args.overdrive,
  force_threshold=args.force, torque_threshold=args.torque)
