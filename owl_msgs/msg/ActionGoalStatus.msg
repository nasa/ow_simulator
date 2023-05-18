# Store and communicate the result state of Ow_Lander actions, organized by action 
#  component/group type. 

Header header

# state value corresponds to value from std_msgs/GoalStatus
actionlib_msgs/GoalStatus[] status_list

# enumeration for indexing and specifying originating goal type
uint16 UNSPECIFIED_GOAL = 0
uint16 ARM_GOAL         = 1
uint16 TASK_GOAL        = 2
uint16 CAMERA_GOAL      = 3
uint16 PAN_TILT_GOAL    = 4
uint16 NUM_GOAL_TYPES   = 5