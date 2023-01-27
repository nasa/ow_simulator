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
server_unstow         = actions.ArmUnstowServer()
server_stow           = actions.ArmStowServer()
server_grind          = actions.GrindServer()
server_dig_circular   = actions.DigCircularServer()
server_dig_linear     = actions.DigLinearServer()
server_discard        = actions.DiscardServer()
server_deliver        = actions.DeliverServer()
server_move_joint     = actions.ArmMoveJointServer()
server_move_joints    = actions.ArmMoveJointsServer()
server_move_cartesian = actions.ArmMoveCartesianServer()
server_move_cartesian_guarded = actions.ArmMoveCartesianGuardedServer()
server_find_surface   = actions.ArmFindSurfaceServer()
server_move_joint_guarded = actions.ArmMoveJointsGuardedServer()

# other non-arm lander actions
server_light_set_intensity = actions.LightSetIntensityServer()
server_camera_capture      = actions.CameraCaptureServer()
server_camera_set_exposure = actions.CameraSetExposureServer()
server_dock_ingest_sample  = actions.DockIngestSampleServer()
server_antenna_pan_tilt    = actions.AntennaPanTiltServer()
server_pan_tilt_move_cartesian = actions.PanTiltMoveCartesianServer()

rospy.spin()
