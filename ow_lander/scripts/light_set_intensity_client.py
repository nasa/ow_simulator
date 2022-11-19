#!/usr/bin/env python3

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import rospy
import ow_lander.msg

from ow_lander.ow_action_client import OWActionClient

def light_set_intensity(name, intensity):
  client = OWActionClient(
    'LightSetIntensity',
    ow_lander.msg.LightSetIntensityAction
  )

  return client.call(
    ow_lander.msg.LightSetIntensityGoal(name = name, intensity = intensity)
  )

if __name__ == '__main__':
  import argparse
  parser = argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    description="Set the intensity of each lander mast spotlight " \
                "individually."
  )
  parser.add_argument(
    'name', type=str, nargs='?', default='left', choices=['left', 'right'],
    help="Light identifier."
  )
  parser.add_argument(
    'intensity', type=float, nargs='?', default=1.0,
    help="Intensity to set the light at. In the range [0.0, 1.0]. " \
         "0 is entirely off, and 1 is max intensity."
  )
  args = parser.parse_args()
  rospy.loginfo(args)
  try:
    rospy.init_node('light_set_intensity_client')
    light_set_intensity(args.name, args.intensity)
  except rospy.ROSInterruptException:
    rospy.logerror("program interrupted before completion")
