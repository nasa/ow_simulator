#!/bin/bash

var=$(ls -r ~/.ros/*.yaml)

rate=5

rostopic pub -r $rate -f $(echo $var | cut -d ' ' -f 6) /shou_yaw_position_controller/command std_msgs/Float64 &
rostopic pub -r $rate -f $(echo $var | cut -d ' ' -f 5) /shou_pitch_position_controller/command std_msgs/Float64 &
rostopic pub -r $rate -f $(echo $var | cut -d ' ' -f 4) /prox_pitch_position_controller/command std_msgs/Float64 &
rostopic pub -r $rate -f $(echo $var | cut -d ' ' -f 3) /dist_pitch_position_controller/command std_msgs/Float64 &
rostopic pub -r $rate -f $(echo $var | cut -d ' ' -f 2) /hand_yaw_position_controller/command std_msgs/Float64 &
rostopic pub -r $rate -f $(echo $var | cut -d ' ' -f 1) /scoop_yaw_position_controller/command std_msgs/Float64 &