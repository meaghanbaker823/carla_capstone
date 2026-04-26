from classes.rule import Rule
from classes.collisionException import CollisionErr

class CollisionRule(Rule):
    """
    The rule subclass for when a car's collision sensor is triggered
    """
    def __init__(self, sensors, car):
        """
        \n\tINPUT(S): sensors: a list of the sensors attached to the vehicle,
                      car: the vehicle object
        \n\tOUTPUT(S): N/A
        """
        super().__init__(sensors, car)
        self.__sensors = sensors
        self.__car = car
        
    def rule_flag(self, traffic_lights):
        """
        Decides if the rule flag should be raised for this cycle (checks if collisions is not empty)
        \n\tINPUT(S): traffic_lights: a list of the traffic lights on the map
        \n\tOUTPUT(S): raises an error if there has been a collision, otherwise returns True
        """
        if(self.__sensors[1].get_collisions() != []):
            raise CollisionErr("")
        return True
    
    def rule_follow(self, traffic_lights):
        """
        Decides how the car should follow this rule when it is triggered (Will tell the car to stop)
        \n\tINPUT(S): traffic_lights: a list of the traffic lights on the map
        \n\tOUTPUT(S): returns 0
        """
        return 0