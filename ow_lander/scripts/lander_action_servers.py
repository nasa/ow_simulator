#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy

from ow_lander import actions
from ow_lander import frame_transformer

rospy.init_node('lander_action_servers')

# handles initialization that must occur after init_node call
frame_transformer.initialize()

# arm actions
server_stop           = actions.ArmStopServer()
server_guarded_move   = actions.GuardedMoveServer()
server_unstow         = actions.ArmUnstowServer()
server_stow           = actions.ArmStowServer()
server_grind          = actions.TaskGrindServer()
server_deliver        = actions.TaskDeliverSampleServer()
server_move_joint     = actions.ArmMoveJointServer()
server_move_joints    = actions.ArmMoveJointsServer()
server_move_cartesian = actions.ArmMoveCartesianServer()
server_move_cartesian_guarded = actions.ArmMoveCartesianGuardedServer()
server_find_surface   = actions.ArmFindSurfaceServer()
server_move_joint_guarded = actions.ArmMoveJointsGuardedServer()
server_dig_circular   = actions.TaskScoopCircularServer()
server_dig_linear     = actions.TaskScoopLinearServer()
server_discard        = actions.TaskDiscardSampleServer()

# other non-arm lander actions
server_light_set_intensity = actions.LightSetIntensityServer()
server_camera_capture      = actions.CameraCaptureServer()
server_camera_set_exposure = actions.CameraSetExposureServer()
server_dock_ingest_sample  = actions.DockIngestSampleServer()
server_antenna_pan_tilt    = actions.PanTiltMoveJointsServer()
server_antenna_pan         = actions.PanServer()
server_antenna_tilt        = actions.TiltServer()
server_pan_tilt_move_cartesian = actions.PanTiltMoveCartesianServer()

rospy.spin()
