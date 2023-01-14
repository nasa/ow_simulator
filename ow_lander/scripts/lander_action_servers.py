#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy

from ow_lander import actions

rospy.init_node('arm_action_servers')

# arm actions
server_stop           = actions.StopServer()
server_guarded_move   = actions.GuardedMoveServer()
server_unstow         = actions.UnstowServer()
server_stow           = actions.StowServer()
server_grind          = actions.GrindServer()
server_dig_circular   = actions.DigCircularServer()
server_dig_linear     = actions.DigLinearServer()
server_discard        = actions.DiscardServer()
server_deliver        = actions.DeliverServer()
server_move_joint     = actions.ArmMoveJointServer()
server_move_joints    = actions.ArmMoveJointsServer()
server_move_cartesian = actions.ArmMoveCartesianServer()
server_arm_set_tool   = actions.ArmSetToolServer()

# other non-arm lander actions
server_light_set_intensity = actions.LightSetIntensityServer()
server_camera_capture      = actions.CameraCaptureServer()
server_dock_ingest_sample  = actions.DockIngestSampleServer()
server_antenna_pan_tilt    = actions.AntennaPanTiltServer()

rospy.spin()
