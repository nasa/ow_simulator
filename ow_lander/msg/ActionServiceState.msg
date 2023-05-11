# Store and communicate the result state of Ow_Lander actions, organized by action 
#  component/group. 

Header header

# component index
uint16 UNSPECIFIED_GOAL = 0
uint16 ARM_GOAL         = 1
uint16 TASK_GOAL        = 2
uint16 CAMERA_GOAL      = 3
uint16 PAN_TILT_GOAL    = 4
uint16 NUM_GOAL_STATES  = 5

# state value corresponds to value from std_msgs/GoalStatus
uint16[] state_array