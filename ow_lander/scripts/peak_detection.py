#!/usr/bin/env python

#This file can be used to tune the three parameters lag, threshold and influence for peak determination. The algorithm takes 3 inputs: lag = the lag of the moving window, threshold = the z-score at which the algorithm signals and influence = the influence (between 0 and 1) of new signals on the mean and standard deviation. This file can be used to change (tune) these numbers on the saved velocity_array for offline peak detection. Once you are satified with the peak detection save the numbers and use them in trajectory publisher.  

# Implementation of algorithm from http://stackoverflow.com/a/22640362/6029703
# Original conde can be found at https://stackoverflow.com/questions/22583391/peak-signal-detection-in-realtime-timeseries-data/43512887#43512887
# modified for our application




import numpy as np
import pylab
import os.path


def thresholding_algo(y, lag, threshold, influence):
    signals = np.zeros(len(y))
    filteredY = np.array(y)
    avgFilter = [0]*len(y)
    stdFilter = [0]*len(y)
    avgFilter[lag - 1] = np.mean(y[0:lag])
    stdFilter[lag - 1] = np.std(y[0:lag])
    for i in range(lag, len(y) - 1):
        if abs(y[i] - avgFilter[i-1]) > threshold * stdFilter [i-1]:
            if y[i] > avgFilter[i-1]:
                signals[i] = 1
            else:
                signals[i] = -1

            filteredY[i] = influence * y[i] + (1 - influence) * filteredY[i-1]
            avgFilter[i] = np.mean(filteredY[(i-lag):i])
            stdFilter[i] = np.std(filteredY[(i-lag):i])
        else:
            signals[i] = 0
            filteredY[i] = y[i]
            avgFilter[i] = np.mean(filteredY[(i-lag):i])
            stdFilter[i] = np.std(filteredY[(i-lag):i])

    return dict(signals = np.asarray(signals),
                avgFilter = np.asarray(avgFilter),
                stdFilter = np.asarray(stdFilter))


filename = os.path.expanduser('~/.ros/velocity_array.txt')
data = np.genfromtxt(filename) 
data = data[~np.isnan(data)]

y = data # use y = data[:800] to truncate the data

#re-tune this number to detect peaks
lag = 100
threshold = 20
influence = 1.0


# Run algo with settings from above
result = thresholding_algo(y, lag=lag, threshold=threshold, influence=influence)

# Plot result
pylab.subplot(311)
pylab.plot(np.arange(1, len(y)+1), y)

pylab.subplot(312)
pylab.plot(np.arange(1, len(y)+1), y)

pylab.plot(np.arange(1, len(y)+1),
           result["avgFilter"], color="cyan", lw=2)
pylab.ylabel('signal')

pylab.plot(np.arange(1, len(y)+1),
           result["avgFilter"] + threshold * result["stdFilter"], color="green", lw=2)

pylab.plot(np.arange(1, len(y)+1),
           result["avgFilter"] - threshold * result["stdFilter"], color="green", lw=2)
#pylab.xlabel('this is x!')
pylab.ylabel('filtered signal with threshold')

pylab.subplot(313)
pylab.step(np.arange(1, len(y)+1), result["signals"], color="red", lw=2)
pylab.ylim(-1.5, 1.5) 
pylab.ylabel('spike detected')
pylab.show()
