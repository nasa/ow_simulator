#!/usr/bin/env python

# __BEGIN_LICENSE__
# Copyright (c) 2018-2019, United States Government as represented by the
# Administrator of the National Aeronautics and Space Administration. All
# rights reserved.
# __END_LICENSE__

import sys
import os
import rospy
import time

def main():
  time.sleep(3)

  # Publish initial position command
  os.system('rostopic pub -1 /shou_yaw_position_controller/command std_msgs/Float64 "data: -1.5" &')
  os.system('rostopic pub -1 /shou_pitch_position_controller/command std_msgs/Float64 "data: 1.5708" &')
  os.system('rostopic pub -1 /prox_pitch_position_controller/command std_msgs/Float64 "data: -2.75" &')
  os.system('rostopic pub -1 /dist_pitch_position_controller/command std_msgs/Float64 "data: 3.1416" &')
  os.system('rostopic pub -1 /hand_yaw_position_controller/command std_msgs/Float64 "data: 0" &')
  os.system('rostopic pub -1 /scoop_yaw_position_controller/command std_msgs/Float64 "data: 0" &')

if __name__ == '__main__':
  main()
