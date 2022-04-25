"""Functions for filtering data

 Last Updated: 02/20/2019
"""

__author__ = "Guru Subramani and Mike Hagenow"

from scipy.signal import butter,filtfilt
import numpy as np

class ButterNotchFilter:
    def __init__(self,order,notch_start,notch_end,fs,num_vars):
        self.order = order
        self.notch_start = notch_start
        self.notch_end = notch_end
        self.fs = fs
        self.num_vars = num_vars
        self.b, self.a = butter(self.order,[self.notch_start, self.notch_end],btype='bandstop',fs=self.fs)

        self.alen = len(self.a)
        self.blen = len(self.b)
        self.xs = []
        self.ys = []
        for ii in range(0,self.blen):
            self.xs.append(np.zeros((self.num_vars)))
        for ii in range(1,self.alen):
            self.ys.append(np.zeros((self.num_vars)))
        
    
    def getFilteredOutput(self,signal):
        if len(signal) != self.num_vars:
            print("FILTERS.PY: trying to filter with wrong length of variables")
            return signal
        val = np.zeros((self.num_vars))
        self.xs.insert(0,signal)
        self.xs.pop(-1)
        for ii in range(0,self.blen):
            val += self.b[ii]*self.xs[ii]
        for ii in range(1,self.alen):
            val -= self.a[ii]*self.ys[ii-1]
        val = val / self.a[0]
        
        self.ys.insert(0,val)
        self.ys.pop(-1)
    
        return val



def butter_lowpass(cutOff, fs, order=5):
    """
        Determines butter filter parameters based on nyquist frequency
        :param cutoff: filter bandwidth (Hz)
        :param fs: sampling frequency (Hz)
        :param order: filter order (magnitude dropoff)
        :type cutOff: float
        :type fs: float
        :type order: int
        :returns: b,a
        :rtype: float, float
    """

    nyq = 0.5 * fs #nyquist half of sampling frequency

    normalCutoff = cutOff / nyq
    b, a = butter(order, normalCutoff, btype='low')
    return b, a


def butter_lowpass_filter(data, cutOff, fs, order=4,axis=0):
    """
        TODO DOCUMENTATION FOR THIS!!!
        :param data: unfiltered data
        :param cutoff: filter bandwidth (Hz)
        :param fs: sampling frequency (Hz)
        :param order: filter order (magnitude dropoff)
        :param axis:
        :type cutOff: float
        :type fs: float
        :type order: int
        :returns: b,a
        :rtype: float, float
    """

    b, a = butter_lowpass(cutOff, fs, order=order)
    y = filtfilt(b, a, data,axis = axis)

    return y

