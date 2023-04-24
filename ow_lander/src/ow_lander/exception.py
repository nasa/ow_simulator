# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

class ActionError(RuntimeError):
  pass

class ArmPlanningError(ActionError):
  """Raise when planning of an arm trajectory has encountered a problem"""
  pass

class ArmExecutionError(ActionError):
  """Raise when execution of an arm trajectory has encountered a problem and
  must be ceased
  """
  pass

class AntennaPlanningError(ActionError):
  """Raise when planning of an antenna trajectory has encountered a problem"""
  pass

class AntennaExecutionError(ActionError):
  """Raise when execution of an antenna trajectory has encountered a problem"""
  pass
