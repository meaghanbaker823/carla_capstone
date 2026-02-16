"""
===========
Rule Class()

__init__(self) creates instance and initilizes attributes
    self.__

function(self) return type | description

===========
"""

class Rule():
    def __init__(self, sensors, car):
        self.__sensors = sensors
        self.__car = car

    def rule_flag(self, traffic_lights):
        pass

    def rule_follow(self, traffic_lights, limit):
        pass