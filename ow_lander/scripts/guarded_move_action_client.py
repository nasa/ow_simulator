#!/usr/bin/env python2

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import actionlib
import ow_lander.msg
import argparse


def guarded_move_client():
    parser = argparse.ArgumentParser()
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
    rospy.loginfo("Requested x_start: %s", args.x_start)
    rospy.loginfo("Requested y_start: %s", args.y_start)
    rospy.loginfo("Requested z_start: %s", args.z_start)
    rospy.loginfo("Requested normal.x: %s", args.direction_x)
    rospy.loginfo("Requested normal.y: %s", args.direction_y)
    rospy.loginfo("Requested normal.z: %s", args.direction_z)
    rospy.loginfo("Requested search distance: %s", args.search_distance)
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
    return client.get_result()  #


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the guarded_move can
        # publish and subscribe over ROS.
        rospy.init_node('guarded_move_client_py')
        result = guarded_move_client()
        rospy.loginfo("Result: %s", result)
    except rospy.ROSInterruptException:
        rospy.logerror("program interrupted before completion")
