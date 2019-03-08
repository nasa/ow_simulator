!/bin/bash
  
if [ "$#" -ne 4 ]; then
    echo "Illegal number of parameters. Usage: <tranch_x> <trench_y> <trench_depth> <output_filename>"
    exit 1
fi

source devel/setup.bash
roslaunch lander_config demo.launch&
sleep 1
disown
sleep 4
rosrun lander_config move_group_python_interface.py $1 $2 $3

