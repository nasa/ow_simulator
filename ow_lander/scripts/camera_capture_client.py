#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import ow_lander.msg

from ow_lander.ow_action_client import OWActionClient

def camera_capture(exposure):
  client = OwActionClient(
    'CameraCapture',
    ow_lander.msg.CameraCaptureAction
  )

  return client.call(
    ow_lander.msg.CameraCaptureGoal(exposure=exposure)
  )

if __name__ == '__main__':
  import argparse
  parser = argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    description="Trigger both mast camera to capture a photograph."
  )
  parser.add_argument(
    'exposure', type=float, nargs='?', default=-1,
    help="Camera exposure in seconds. If <= 0 the previous exposure setting " \
         "will be used."
  )
  args = parser.parse_args()
  rospy.loginfo(args)
  try:
    # Initializes a rospy node
    rospy.init_node('camera_capture_client')
    camera_capture()
  except rospy.ROSInterruptException:
    rospy.logerror('program interrupted before completion')

