class SpeedMonitor():
    def __init__(self, threshold):
        self.__threshold = threshold

    def get_threshold(self):
        return self.__threshold