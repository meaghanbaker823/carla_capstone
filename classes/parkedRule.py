from classes.rule import Rule

"""
===========
parkedRule Class(Rule)

__init__(self) creates instance and initilizes attributes
    self.__

function(self) return type | description

===========
"""

class ParkedRule(Rule):
    def __init__(self, sensors, car):
        super().__init__(sensors, car)
        self.__sensors = sensors
        self.__car = car
        
    def rule_flag(self, traffic_lights):
        if(self.__sensors[0].get_other_actors() != []):

            if(self.__sensors[0].get_detections()[-1][0].type_id[:-2].startswith('vehicle')):
                return False
        return True
    
    def rule_follow(self, traffic_lights):
        # pass brake as 1, speed as 0, and steering as None

        return 1