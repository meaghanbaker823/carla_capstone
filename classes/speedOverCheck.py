from classes.speedMonitor import SpeedMonitor

"""
===========
SpeedOverCheck Class(SpeedMonitor)

__init__(self) creates instance and initilizes attributes
    self.__

function(self) return type | description

===========
"""

class SpeedOverCheck(SpeedMonitor):
    def speed_check(self, current, limit):
        if(current >= limit):
            return 0
        return -1