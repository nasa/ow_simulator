#!/usr/bin/env python

# __BEGIN_LICENSE__
# Copyright (c) 2018-2019, United States Government as represented by the
# Administrator of the National Aeronautics and Space Administration. All
# rights reserved.
# __END_LICENSE__

import sys
import os
import rospy

def main():
  # Publish initial position command
  os.system("rostopic pub -1 /shou_yaw_position_controller/command std_msgs/Float64 -1.5&")
  os.system("rostopic pub -1 /shou_pitch_position_controller/command std_msgs/Float64 1.5708&")
  os.system("rostopic pub -1 /prox_pitch_position_controller/command std_msgs/Float64 -2.75&")
  os.system("rostopic pub -1 /dist_pitch_position_controller/command std_msgs/Float64 3.1416&")
  os.system("rostopic pub -1 /hand_yaw_position_controller/command std_msgs/Float64 0&")
  os.system("rostopic pub -1 /scoop_yaw_position_controller/command std_msgs/Float64 0&")

if __name__ == '__main__':
  main()