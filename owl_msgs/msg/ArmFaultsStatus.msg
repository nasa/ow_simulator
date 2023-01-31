# Arm fault flags

Header header
uint64 value

uint64 NONE                  = 0
uint64 HARDWARE              = 1
uint64 TRAJECTORY_GENERATION = 2
uint64 COLLISION             = 4
uint64 E_STOP                = 8
uint64 POSITION_LIMIT        = 16
uint64 JOINT_TORQUE_LIMIT    = 32
uint64 VELOCITY_LIMIT        = 64
uint64 NO_FORCE_DATA         = 128
uint64 FORCE_TORQUE_LIMIT    = 256
