#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
import ow_lander.msg
import argparse
from guarded_move_action_client import print_arguments

def stow_client():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        'default', type=float, help='This action client Stows the lander arm', nargs='?', default=0, const=0)
    args = parser.parse_args()
    print_arguments(args)
    client = actionlib.SimpleActionClient('Stow', ow_lander.msg.UnstowAction)

    client.wait_for_server()
    #dummy goal for action server
    goal = ow_lander.msg.UnstowGoal(goal=20)

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
        # Initializes a rospy node so that the StowActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('stow_client_py')
        result = stow_client()
        rospy.loginfo("Result: %s", result)
    except rospy.ROSInterruptException:
        rospy.logerror("program interrupted before completion")
