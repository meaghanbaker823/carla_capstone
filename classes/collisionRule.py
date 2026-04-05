from classes.rule import Rule
from classes.collision_exception import CollisionErr
"""
===========
CollisionRule Class(Rule)

__init__(self) creates instance and initilizes attributes
    self.__

function(self) return type | description

===========
"""

class CollisionRule(Rule):
    def __init__(self, sensors, car):
        super().__init__(sensors, car)
        self.__sensors = sensors
        self.__car = car
        
    def rule_flag(self, traffic_lights):
        if(self.__sensors[1].get_collisions() != []):
            raise CollisionErr("")
        return True
    
    def rule_follow(self, traffic_lights):
        # pass brake as 1, speed as 0, and steering as None
        return 0