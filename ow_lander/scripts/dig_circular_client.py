#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

# Default trenching values
# goal.x_start = 1.65
# goal.y_start = 0
# goal.depth = 0.01
# goal.parallel = False
# goal.ground_position = constants.DEFAULT_GROUND_HEIGHT

# General trenching values
# goal.x_start = 1.55
# goal.y_start = 0.2
# goal.depth = 0.045
# goal.parallel = False
# goal.ground_position = -0.155

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
import ow_lander.msg
import constants
import argparse
from guarded_move_action_client import print_arguments


def DigCircular_client():

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('x_start', type=float,
                        help='X coordinate of trenching starting point', nargs='?', default=1.65, const=0)
    parser.add_argument('y_start', type=float,
                        help='Y coordinate of trenching starting point', nargs='?', default=0.0, const=0)
    parser.add_argument('depth', type=float,
                        help='Desired depth', nargs='?', default=0.01, const=0)
    parser.add_argument('parallel', type=lambda x: eval(x[0].upper() + x[1:].lower()),
                        help='If True, resulting trench is parallel to arm. If False, perpendicular to arm', nargs='?', default= True)
    parser.add_argument('ground_position', type=float, help='Z coordinate of ground level in base_link frame',
                        nargs='?', default=constants.DEFAULT_GROUND_HEIGHT, const=0)
    args = parser.parse_args()
    print_arguments(args)

    client = actionlib.SimpleActionClient(
        'DigCircular', ow_lander.msg.DigCircularAction)

    client.wait_for_server()

    goal = ow_lander.msg.DigCircularGoal()

    goal.x_start = args.x_start
    goal.y_start = args.y_start
    goal.depth = args.depth
    goal.parallel = args.parallel
    goal.ground_position = args.ground_position

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
        # Initializes a rospy node so that the dig circular client can
        # publish and subscribe over ROS.
        rospy.init_node('digCircular_client_py')
        result = DigCircular_client()
        rospy.loginfo("Result: %s", result)
    except rospy.ROSInterruptException:
        rospy.logerror("program interrupted before completion")
