#!/usr/bin/env python

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

from __future__ import print_function

import rospy
import actionlib
import ow_lander.msg
import constants


def Grind_client():
 
    client = actionlib.SimpleActionClient('Grind', ow_lander.msg.GrindAction)

    client.wait_for_server()

    goal = ow_lander.msg.GrindGoal()

    # Default trenching values
    
    goal.x_start = 1.65
    goal.y_start = 0.0
    goal.depth = 0.05
    goal.length = 0.6 # 0.6
    goal.parallel = True
    goal.ground_position = constants.DEFAULT_GROUND_HEIGHT

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
        rospy.init_node('Grind_client_py')
        result = Grind_client()
        print("Result:", ', ',result)
        #print("Result:", ', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
