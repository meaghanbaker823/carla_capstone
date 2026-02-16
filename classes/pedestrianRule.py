from classes.rule import Rule

"""
===========
PedestrianRule Class()

__init__(self) creates instance and initilizes attributes
    self.__

function(self) return type | description

===========
"""

class PedestrianRule(Rule):
    def __init__(self, sensors, car):
        super().__init__(sensors, car)
        self.__sensors = sensors
        self.__car = car

    def rule_flag(self, walkers):
        if (walkers.react_to_walkers(self.__car.get()) != ""):
            if(walkers.get_response()== "stop"):
                self.__car.stop_car()
            else:
                self.__car.drive
            return False
        return True
    

    """

class ObstacleRule(Rule):
    def __init__(self, sensors, car):
        super().__init__(sensors, car)
        self.__sensors = sensors
        self.__car = car

    def rule_flag(self, traffic_lights):
        if(self.__sensors[0].get_other_actors() != []):
            self.__car.avoid_obstacles()
            return False
        return True
    """