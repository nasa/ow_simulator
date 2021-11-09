#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import actionlib
from math import pi
import argparse
import ow_lander.msg
from guarded_move_action_client import print_arguments


def wrap_angle(angle):
    """
    :param angle: (float)
    :return: (float) the angle in [-pi, pi]
    """
    tolerance = 0.01
    while angle > (pi+tolerance):
        angle -= 2 * pi
    while angle < -(pi-tolerance):
        angle += 2 * pi
    return angle


def antenna_client():

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        'pan', type=float, help='Antenna pan value in radians', nargs='?', default=0, const=0)
    parser.add_argument(
        'tilt', type=float, help='Antenna tilt value in radians', nargs='?', default=0, const=0)
    args = parser.parse_args()
    print_arguments(args)

    client = actionlib.SimpleActionClient(
        'AntennaPanTiltAction', ow_lander.msg.AntennaPanTiltAction)
    client.wait_for_server()

    goal = ow_lander.msg.AntennaPanTiltGoal()
    goal.pan = args.pan
    goal.tilt = args.tilt

    goal.tilt = wrap_angle(goal.tilt)
    goal.pan = wrap_angle(goal.pan)

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
        rospy.loginfo("Result: %s", result)
    except rospy.ROSInterruptException:
        rospy.logerror("program interrupted before completion")
