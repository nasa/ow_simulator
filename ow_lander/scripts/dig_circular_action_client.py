#!/usr/bin/env python2

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import actionlib
import ow_lander.msg
import constants
import argparse


def DigCircular_client():

    parser = argparse.ArgumentParser()
    parser.add_argument('x_start', type=float,
                        help='X coordinate of trenching starting point', nargs='?', default=1.65, const=0)
    parser.add_argument('y_start', type=float,
                        help='Y coordinate of trenching starting point', nargs='?', default=0.0, const=0)
    parser.add_argument('depth', type=float,
                        help='Desired depth', nargs='?', default=0.01, const=0)
    parser.add_argument('parallel', type=bool,
                        help='If True, resulting trench is parallel to arm. If False, perpendicular to arm', nargs='?', default=0, const=0)
    parser.add_argument('ground_position', type=float, help='Z coordinate of ground level in base_link frame',
                        nargs='?', default=constants.DEFAULT_GROUND_HEIGHT, const=0)
    args = parser.parse_args()
    rospy.loginfo("Requested x_start: %s", args.x_start)
    rospy.loginfo("Requested y_start: %s", args.y_start)
    rospy.loginfo("Requested depth: %s", args.depth)
    rospy.loginfo("Requested parallel: %s", args.parallel)
    rospy.loginfo("Requested ground_position: %s", args.ground_position)

    client = actionlib.SimpleActionClient(
        'DigCircular', ow_lander.msg.DigCircularAction)

    client.wait_for_server()

    goal = ow_lander.msg.DigCircularGoal()

    goal.x_start = args.x_start
    goal.y_start = args.y_start
    goal.depth = args.depth
    goal.parallel = args.parallel
    goal.ground_position = args.ground_position
    # Default trenching values
    # goal.x_start = 1.65
    # goal.y_start = 0
    # goal.depth = 0.01
    # goal.parallel = False
    # goal.ground_position = constants.DEFAULT_GROUND_HEIGHT

    # General trenching values
    #goal.x_start = 1.55
    #goal.y_start = 0.2
    #goal.depth = 0.045
    #goal.parallel = False
    #goal.ground_position = -0.155

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  #


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the UnstowActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('digCircular_client_py')
        result = DigCircular_client()
        rospy.loginfo("Result: %s", result)
    except rospy.ROSInterruptException:
        rospy.logerror("program interrupted before completion")
