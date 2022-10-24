#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import actionlib
import argparse
import ow_lander.msg
from constants import PAN_MIN, PAN_MAX, PAN_TOLERANCE
from constants import TILT_MIN, TILT_MAX, TILT_TOLERANCE
from utils import normalize_radians, in_range
from guarded_move_action_client import print_arguments

def antenna_client():

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        'pan',
        type=float,
        help='Antenna pan value in radians (-3.2 - 3.2)',
        nargs='?', default=0, const=0)
    parser.add_argument(
        'tilt',
        type=float,
        help='Antenna tilt value in radians (-pi/2 - pi/2)',
        nargs='?', default=0, const=0)
    args = parser.parse_args()
    print_arguments(args)

    if not in_range(args.pan, PAN_MIN, PAN_MAX):
        rospy.logwarn('Requested pan %s not within allowed limit.' % args.pan)
        return
    if not in_range(args.tilt, TILT_MIN, TILT_MAX):
        rospy.logwarn('Requested tilt %s not within allowed limit.' % args.tilt)
        return

    client = actionlib.SimpleActionClient(
        'AntennaPanTiltAction', ow_lander.msg.AntennaPanTiltAction)
    client.wait_for_server()

    goal = ow_lander.msg.AntennaPanTiltGoal()
    goal.pan = args.pan
    goal.tilt = args.tilt

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()


if __name__ == '__main__':
    try:
        # Initializes a rospy node

        rospy.init_node('antenna_client_py')
        result = antenna_client()
        rospy.loginfo("Result from server: %s", result)
    except rospy.ROSInterruptException:
        rospy.logerror("program interrupted before completion")
