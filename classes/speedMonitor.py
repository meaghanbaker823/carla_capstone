"""
===========
SpeedMonitor Class()

__init__(self) creates instance and initilizes attributes
    self.__

function(self) return type | description

===========
"""

class SpeedMonitor():
    def __init__(self, threshold):
        self.__threshold = threshold

    def get_threshold(self):
        return self.__threshold