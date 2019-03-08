#!/bin/bash

if [ "$#" -ne 1 ]; then
    echo "Illegal number of parameters. Usage: <moveit_out_timestamp.bag>"
    exit 1
fi

rostopic pub /scoop_yaw_position_controller/command std_msgs/Float64 -f joint_topic.txt -r 0.1 &
rostopic pub /hand_yaw_position_controller/command std_msgs/Float64 -f joint_topic.txt -r 0.1
