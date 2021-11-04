#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

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

    # General trenching values for non- parallel circular/linear trenching     
    #goal.x_start = 1.55
    #goal.y_start = 0.1
    #goal.depth = 0.045
    #goal.length = 0.5 # 0.6 
    #goal.parallel = False
    #goal.ground_position = -0.155

    # General trenching values for linear trenching            
    #goal.x_start = 1.65
    #goal.y_start = 0.2
    #goal.depth = 0.045
    ##choose a generous value for goal length to avoid collision
    #goal.length = 0.7  
    #goal.parallel = True
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
        rospy.init_node('Grind_client_py')
        result = Grind_client()
        rospy.loginfo("Result: %s", result)
    except rospy.ROSInterruptException:
        rospy.logerror("program interrupted before completion")
