#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import actionlib
import ow_lander.msg

def unstow_client():
 
    client = actionlib.SimpleActionClient('Unstow', ow_lander.msg.UnstowAction)

    client.wait_for_server()
    #dummy goal for action server
    goal = ow_lander.msg.UnstowGoal(goal=20)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the UnstowActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('unstow_client_py')
        result = unstow_client()
        rospy.loginfo("Result: %s", result)
    except rospy.ROSInterruptException:
        rospy.logerror("program interrupted before completion")
