#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy

from ow_actions.actions import unstow

rospy.init_node('arm_action_servers_experimental')
server_unstow = unstow.UnstowServer()

# TODO: other actions go here

rospy.spin()
