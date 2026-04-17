"""
===========
AngleConstraint Class()

__init__(self) creates instance and initilizes attributes
    self.__

function(self) return type | description

===========
"""

class AngleConstraint():
    def __init__(self, min, max, adjust):
        self.__min = min
        self.__max = max
        self.__adjust = adjust

    def clamp(self, degrees):
        if degrees < self.__min:
            return degrees - self.__adjust
        elif degrees > self.__max:
            return degrees - self.__adjust

        return degrees
    
    def max_steer(self, degrees):
        if degrees < self.__min:
            return self.__min
        elif degrees > self.__max:
            return self.__max
        else:
            return degrees    