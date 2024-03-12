class RollingAverage:
    def __init__(self, window_size):
        self.data = []
        self.window_size = window_size

    def add(self, value):
        self.data.append(value)
        if len(self.data) > self.window_size:
            self.data.pop(0)  # remove the oldest value if we've exceeded the window size

    def average(self):
        return sum(self.data) / len(self.data) if self.data else None
    
    def is_full(self):
        return len(self.data) == self.window_size