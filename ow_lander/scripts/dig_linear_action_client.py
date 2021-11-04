#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

# Default trenching values
# goal.x_start = 1.46
# goal.y_start = 0
# goal.depth = 0.01
# goal.length = 0.1
# goal.ground_position = constants.DEFAULT_GROUND_HEIGHT

# General trenching values
# goal.x_start = 1.45
# goal.y_start = 0.2
# goal.depth = 0.045
# goal.length = 0.1 #
# goal.ground_position = -0.155

import rospy
import actionlib
import ow_lander.msg
import constants
import argparse


def DigLinear_client():

    parser = argparse.ArgumentParser()
    parser.add_argument('x_start', type=float,
                        help='X coordinate of trenching starting point', nargs='?', default=1.46, const=0)
    parser.add_argument('y_start', type=float,
                        help='Y coordinate of trenching starting point', nargs='?', default=0.0, const=0)
    parser.add_argument('depth', type=float,
                        help='Desired depth', nargs='?', default=0.01, const=0)
    parser.add_argument('length', type=float,
                        help='Desired length', nargs='?', default=0.1, const=0)
    parser.add_argument('parallel', type=bool,
                        help='If True, resulting trench is parallel to arm. If False, perpendicular to arm', nargs='?', default=1, const=0)
    parser.add_argument('ground_position', type=float, help='Z coordinate of ground level in base_link frame',
                        nargs='?', default=constants.DEFAULT_GROUND_HEIGHT, const=0)
    args = parser.parse_args()
    rospy.loginfo("Requested x_start: %s", args.x_start)
    rospy.loginfo("Requested y_start: %s", args.y_start)
    rospy.loginfo("Requested depth: %s", args.depth)
    rospy.loginfo("Requested length: %s", args.length)
    rospy.loginfo("Requested parallel: %s", args.parallel)
    rospy.loginfo("Requested ground_position: %s", args.ground_position)

    client = actionlib.SimpleActionClient(
        'DigLinear', ow_lander.msg.DigLinearAction)

    client.wait_for_server()

    goal = ow_lander.msg.DigLinearGoal()

    goal.x_start = args.x_start
    goal.y_start = args.y_start
    goal.depth = args.depth
    goal.length = args.length
    goal.ground_position = args.ground_position

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  #


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the deliver client can
        # publish and subscribe over ROS.
        rospy.init_node('digLinear_client_py')
        result = DigLinear_client()
        rospy.loginfo("Result: %s", result)
    except rospy.ROSInterruptException:
        rospy.logerror("program interrupted before completion")
