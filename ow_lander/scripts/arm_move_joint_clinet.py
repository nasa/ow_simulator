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


def discard_client():

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('relative', type=float,
                        help='Move joint relative to the current position ', nargs='?', default= 0, const=0)
    parser.add_argument('joint', type=float,
                        help='Joint index to be move', nargs='?', default=2, const=0)
    parser.add_argument('angle', type=float,
                        help='goal angle of the joint in radians', nargs='?', default=0.5, const=0)
    args = parser.parse_args()
    print_arguments(args)

    client = actionlib.SimpleActionClient(
        'ArmMoveJoint', ow_lander.msg.ARM_MOVE_JOINTAction)

    client.wait_for_server()

    goal = ow_lander.msg.ARM_MOVE_JOINTGoal()

    goal.relative = args.relative
    goal.joint = args.joint
    goal.angle = args.angle

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
        # Initializes a rospy node so that the discard client can
        # publish and subscribe over ROS.
        rospy.init_node('discard_client_py')
        result = discard_client()
        rospy.loginfo("Result: %s", result)
    except rospy.ROSInterruptException:
        rospy.logerror("program interrupted before completion")
