from classes.speedMonitor import SpeedMonitor

"""
===========
SpeedPerfectCheck Class(SpeedMonitor)

__init__(self) creates instance and initilizes attributes
    self.__

function(self) return type | description

===========
"""

class SpeedPerfectCheck(SpeedMonitor):
    def speed_check(self, current, limit):
        if(current == (limit - self.get_threshold())):
            return 0.3
        return -1