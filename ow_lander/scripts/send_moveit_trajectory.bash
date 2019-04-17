#!/bin/bash

# __BEGIN_LICENSE__
# Copyright (c) 2018-2019, United States Government as represented by the
# Administrator of the National Aeronautics and Space Administration. All
# rights reserved.
# __END_LICENSE__

# Get all the yaml files from .ros
var=$(ls -r ~/.ros/*.yaml)

# rostopic pub rate
rate=5

# Extract the 6 most recent trajectory yaml files and publishes them on the appropriate topics
rostopic pub -r $rate -f $(echo $var | cut -d ' ' -f 6) /shou_yaw_position_controller/command std_msgs/Float64 &
rostopic pub -r $rate -f $(echo $var | cut -d ' ' -f 5) /shou_pitch_position_controller/command std_msgs/Float64 &
rostopic pub -r $rate -f $(echo $var | cut -d ' ' -f 4) /prox_pitch_position_controller/command std_msgs/Float64 &
rostopic pub -r $rate -f $(echo $var | cut -d ' ' -f 3) /dist_pitch_position_controller/command std_msgs/Float64 &
rostopic pub -r $rate -f $(echo $var | cut -d ' ' -f 2) /hand_yaw_position_controller/command std_msgs/Float64 &
rostopic pub -r $rate -f $(echo $var | cut -d ' ' -f 1) /scoop_yaw_position_controller/command std_msgs/Float64 &
