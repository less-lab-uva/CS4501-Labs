from collections import deque
# The MovingAverage implements the concept of window in a set of measurements.
# The window_size is the number of most recent measurements used in average.
# The measurements are continuously added using the method add.
# The get_average method must return the average of the last window_size measurements.
class MovingAverage:

    def __init__(self, window_size):
        self.window_size = window_size
        self.data = deque(maxlen=self.window_size)

    # add a new measurement
    def add(self, val):
        self.data.append(val)

    # return the average of the last window_size measurements added
    # or the average of all measurements if less than window_size were provided
    def get_average(self):
        return sum(self.data) / self.window_size
