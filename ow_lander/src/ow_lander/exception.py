# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

class ArmError(RuntimeError):
  pass

class ArmPlanningError(ArmError):
  """Raise when planning of an arm trajectory has encountered a problem"""
  pass

class ArmExecutionError(ArmError):
  """Raise when execution of an arm trajectory has encountered a problem and
  must be ceased
  """
  pass

class AntennaError(RuntimeError):
  pass

class AntennaPlanningError(AntennaError):
  """Raise when planning of an antenna trajectory has encountered a problem"""
  pass

class AntennaExecutionError(AntennaError):
  """Raise when execution of an antenna trajectory has encountered a problem"""
  pass
