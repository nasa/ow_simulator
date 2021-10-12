# System Fault Status Mask
# Value is the fault classification. Internally within faults, we use a bitmask to set the 
# different faults such that different faults can occur at the same time (ie. arm and antenna). 
# To translate the bitmask in the message, we translate the bitmask value to an integer to send 
# it to the corresponding topic. See bitmask in FaultInjector.h for implementation of the following:
# System=1, 
# ArmGoalError=2, 
# ArmExecutionError=4,
# TaskGoalError=8, 
# CamGoalError=16, 
# CamExecutionError=32, 
# PtGoalError=64, 
# PtExecutionError=128, 
# LanderExecutionError = 256,
# PowerExecutionError = 512
Header header
uint64 value

