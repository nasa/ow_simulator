#!/usr/bin/env python2

# The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
# Research and Simulation can be found in README.md in the root directory of
# this repository.

import numpy as np
import csv
from collections import deque

class SlidingWindow:
  """
  A class that maintains a sliding window over a stream of samples in FIFO order
  """

  def __init__(self, size, method):
    self.que = deque()
    self.size = size
    self.method = method

  def append(self, val):
    if len(self.que) >= self.size:
      self.que.pop()
    self.que.appendleft(val)

  @property
  def value(self):
    # TODO: consider caching the value to speed up query
    return self.method(self.que)


class PeakDetectionRT:
  """
  This class implements a modified version of the peak detection algorithm described in:
  https://stackoverflow.com/questions/22583391/peak-signal-detection-in-realtime-timeseries-data
  This implementation acts on a data stream rather than assuming all data preloaded
  into an array.
  """

  def __init__(self, lag, threshold, influence):
    self.lag = lag
    self.threshold = threshold
    self.influence = influence

    self.reset()

  def reset(self):
    """ Resets all internal counters.
    Use for every new session without having to recreate the object. """
    self.counter = 0
    self.y_raw = 0
    self.y_filtered = 0
    self.signal = 0
    self.avg_filter = SlidingWindow(self.lag, np.mean)
    self.std_filter = SlidingWindow(self.lag, np.std)


  def _warmup(self):
    """
    accumulate enough samples (determined by lag) before performing detection 
    :return: always returns zero. i.e. no peak or anomaly.
    """
    self.y_filtered = self.y_raw
    self.avg_filter.append(self.y_filtered)
    self.std_filter.append(self.y_filtered)
    return self.signal  # signal is 0

  def _detect(self):
    if abs(self.y_raw - self.avg_filter.value) > self.threshold * self.std_filter.value:
      self.signal = np.sign(self.y_raw - self.avg_filter.value)
      self.y_filtered = self.influence * self.y_raw + \
          (1 - self.influence) * self.y_filtered
    else:
      self.signal = 0
      self.y_filtered = self.y_raw
    self.avg_filter.append(self.y_filtered)
    self.std_filter.append(self.y_filtered)
    return self.signal

  def detect(self, value):
    """
    Performs peak detection

    :param float value: a single value of a stream of data
    :return: If a peak is detected the method returns +1 or -1 which indicate
    a strong postive deivation from the mean or negative one respectively. When
    no peak is detected the method returns 0.
    :rtype: int
    """
    self.counter += 1
    self.y_raw = value
    result = self._warmup() if self.counter < self.lag else self._detect()
    return result
