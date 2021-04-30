#!/usr/bin/env python

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.
from __future__ import print_function
import rospy
import actionlib
import ow_lander.msg
import sys


def guarded_move_client():
 
    client = actionlib.SimpleActionClient('GuardedMove', ow_lander.msg.GuardedMoveAction)

    client.wait_for_server()

    goal = ow_lander.msg.GuardedMoveGoal()
    
    # Default trenching values
    goal.start.x = 2.0
    goal.start.y = 0.0
    goal.start.z = 0.3
    goal.normal.x = 0.0
    goal.normal.y = 0.0
    goal.normal.z = 1.0
    goal.search_distance = 0.5

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
        rospy.init_node('guarded_move_client_py')
        result = guarded_move_client()
        print("Result:", ', ',result)
        #print("Result:", ', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
