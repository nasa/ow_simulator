# System Fault Status Mask
# Value is the fault classification. See bitmask in FaultInjector.h for implementation of the following:
# System=1, 
# ArmGoalError=2, 
# ArmExecutionError=4,
# TaskGoalError=8, 
# CamGoalError=16, 
# CamExecutionError=32, 
# PtGoalError=64, 
# PtExecutionError=128, 
# LanderExecutionError = 256
Header header
uint64 value

