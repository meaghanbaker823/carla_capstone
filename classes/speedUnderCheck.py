from classes.speedMonitor import SpeedMonitor

"""
===========
SpeedUnderCheck Class(SpeedMonitor)

__init__(self) creates instance and initilizes attributes
    self.__

function(self) return type | description

===========
"""

class SpeedUnderCheck(SpeedMonitor):
    def speed_check(self, current, limit):
        if(current < (limit - self.get_threshold())):
            return 0.8
        return -1