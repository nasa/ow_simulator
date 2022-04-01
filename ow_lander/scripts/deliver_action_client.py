#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

# Default trenching values
# goal.delivery.x = 0.55
# goal.delivery.y = -0.3
# goal.delivery.z = 0.82

# Remove Tailings values
# goal.delivery.x = 1.5
# goal.delivery.y = 0.8
# goal.delivery.z = 0.65

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
import ow_lander.msg
import argparse
from guarded_move_action_client import print_arguments


def deliver_client():

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('x_start', type=float,
                        help='x coordinates of sample delivery location', nargs='?', default=0.55, const=0)
    parser.add_argument('y_start', type=float,
                        help='y coordinates of sample delivery location', nargs='?', default=-0.3, const=0)
    parser.add_argument('z_start', type=float,
                        help='z coordinates of sample delivery location', nargs='?', default=0.82, const=0)
    args = parser.parse_args()
    print_arguments(args)

    client = actionlib.SimpleActionClient(
        'Deliver', ow_lander.msg.DeliverAction)

    client.wait_for_server()

    goal = ow_lander.msg.DeliverGoal()

    goal.delivery.x = args.x_start
    goal.delivery.y = args.y_start
    goal.delivery.z = args.z_start

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
        # Initializes a rospy node so that the deliver client can
        # publish and subscribe over ROS.
        rospy.init_node('deliver_client_py')
        result = deliver_client()
        rospy.loginfo("Result: %s", result)
    except rospy.ROSInterruptException:
        rospy.logerror("program interrupted before completion")
