#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import actionlib
# import argparse
import ow_lander.msg
# from guarded_move_action_client import print_arguments

def dock_ingest_sample_client():
    # parser = argparse.ArgumentParser(
    #     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    # args = parser.parse_args()
    # print_arguments(args)

    rospy.loginfo("waiting for action")

    client = actionlib.SimpleActionClient(
        'DockIngestSampleAction', ow_lander.msg.DockIngestSampleAction)
    client.wait_for_server()

    goal = ow_lander.msg.DockIngestSampleGoal()

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()

if __name__ == '__main__':
    try:
        # Initializes a rospy node
        rospy.init_node('dock_ingest_sample_client_py')
        result = dock_ingest_sample_client()
        rospy.loginfo("Result: %s", result)
    except rospy.ROSInterruptException:
        rospy.logerror("program interrupted before completion")
