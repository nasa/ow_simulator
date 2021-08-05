#!/usr/bin/env python2

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

from __future__ import print_function

import rospy
import actionlib
import ow_lander.msg


def antenna_client():
 
    client = actionlib.SimpleActionClient('AntennaPanTiltAction', ow_lander.msg.AntennaAction)

    client.wait_for_server()

    goal = ow_lander.msg.AntennaGoal()

    goal.pan = 1.5
    goal.tilt = 1.5 

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
        rospy.init_node('antenna_client_py')
        result = antenna_client()
        print("Result:", ', ',result)
        #print("Result:", ', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
