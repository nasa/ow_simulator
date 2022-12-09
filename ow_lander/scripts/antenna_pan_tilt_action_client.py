#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import actionlib
import argparse
import ow_lander.msg
from ow_lander.constants import PAN_MIN, PAN_MAX, TILT_MIN, TILT_MAX, PAN_TILT_INPUT_TOLERANCE
from ow_lander.common import in_closed_range
from guarded_move_action_client import print_arguments

def antenna_client():

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('pan',
                        type=float,
                        help='Antenna pan value in radians [-3.2, 3.2]',
                        nargs='?', default=0)
    parser.add_argument('tilt',
                        type=float,
                        help='Antenna tilt value in radians [-1.56, 1.56]',
                        nargs='?', default=0)
    args = parser.parse_args()
    print_arguments(args)

    if not in_closed_range(args.pan, PAN_MIN, PAN_MAX, PAN_TILT_INPUT_TOLERANCE):
        rospy.logwarn('Requested pan %s not within allowed limit.' % args.pan)
        return None
    if not in_closed_range(args.tilt, TILT_MIN, TILT_MAX,
                           PAN_TILT_INPUT_TOLERANCE):
        rospy.logwarn('Requested tilt %s not within allowed limit.' % args.tilt)
        return None

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
