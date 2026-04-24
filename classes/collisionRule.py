from classes.rule import Rule
from classes.collisionException import CollisionErr

class CollisionRule(Rule):
    """
    The rule subclass for when a car's collision sensor is triggered
    """
    def __init__(self, sensors, car):
        """
        Sets up the variables needed for the CollisionRule class
        \n\tINPUT(S):
        \n\tOUTPUT(S):
        """
        super().__init__(sensors, car)
        self.__sensors = sensors
        self.__car = car
        
    def rule_flag(self, traffic_lights):
        """
        Decides if the rule flag should be raised for this cycle (checks if collisions is not empty)
        \n\tINPUT(S):
        \n\tOUTPUT(S):
        """
        if(self.__sensors[1].get_collisions() != []):
            raise CollisionErr("")
        return True
    
    def rule_follow(self, traffic_lights):
        """
        Decides how the car should follow this rule when it is triggered (Will tell the car to stop)
        \n\tINPUT(S):
        \n\tOUTPUT(S):     
        """
        return 0