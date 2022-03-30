"""Functions for filtering data

 Last Updated: 02/20/2019
"""

__author__ = "Guru Subramani and Mike Hagenow"

from scipy.signal import butter,filtfilt


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

