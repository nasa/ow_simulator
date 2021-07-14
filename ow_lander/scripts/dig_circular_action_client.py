#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

from __future__ import print_function

import rospy
import actionlib
import ow_lander.msg
import constants


def DigCircular_client():
 
    client = actionlib.SimpleActionClient('DigCircular', ow_lander.msg.DigCircularAction)

    client.wait_for_server()

    goal = ow_lander.msg.DigCircularGoal()

    # Default trenching values
    goal.x_start = 1.65
    goal.y_start = 0
    goal.depth = 0.01
    goal.parallel = True
    goal.ground_position = constants.DEFAULT_GROUND_HEIGHT
    
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
        print("Result:", ', ',result)
        #print("Result:", ', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
