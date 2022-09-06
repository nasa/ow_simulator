#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
import ow_lander.msg
import argparse

def print_arguments(args):
    # convert from namespace to dict and print pairs
    for name, value in zip(vars(args).keys(), vars(args).values()):
        rospy.loginfo("Requested %s value: %s" % (name, value))

def guarded_move_client():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('x_start', type=float,
                        help='x coordinates of guarded move starting point', nargs='?', default=2.0, const=0)
    parser.add_argument('y_start', type=float,
                        help='y coordinates of guarded move starting point', nargs='?', default=0.0, const=0)
    parser.add_argument('z_start', type=float,
                        help='z coordinates of guarded move starting point', nargs='?', default=0.3, const=0)
    parser.add_argument('direction_x', type=float,
                        help='x coordinates of the vector normal to the surface', nargs='?', default=0.0, const=0)
    parser.add_argument('direction_y', type=float,
                        help='y coordinates of the vector normal to the surface', nargs='?', default=0.0, const=0)
    parser.add_argument('direction_z', type=float,
                        help='z coordinates of the vector normal to the surface', nargs='?', default=1.0, const=0)
    parser.add_argument('search_distance', type=float,
                        help='Search Distance', nargs='?', default=0.5, const=0)
    args = parser.parse_args()
    print_arguments(args)
    client = actionlib.SimpleActionClient(
        'GuardedMove', ow_lander.msg.GuardedMoveAction)

    client.wait_for_server()

    goal = ow_lander.msg.GuardedMoveGoal()

    goal.start.x = args.x_start
    goal.start.y = args.y_start
    goal.start.z = args.z_start
    goal.normal.x = args.direction_x
    goal.normal.y = args.direction_y
    goal.normal.z = args.direction_z
    goal.search_distance = args.search_distance

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    if client.get_state() == GoalStatus.ABORTED:
        return ('aborted')
    else: 
        return client.get_result()


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the guarded_move can
        # publish and subscribe over ROS.
        rospy.init_node('guarded_move_client_py')
        result = guarded_move_client()
        rospy.loginfo("Result: %s", result)
    except rospy.ROSInterruptException:
        rospy.logerror("program interrupted before completion")
