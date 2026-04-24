from classes.rule import Rule

class ParkedRule(Rule):
    """
    The rule subclass for when a car's obstacle detector is triggered by a car
    """
    def __init__(self, sensors, car):
        """
        Sets up the variables needed for the ParkedRule class
        \n\tINPUT(S):
        \n\tOUTPUT(S):
        """
        super().__init__(sensors, car)
        self.__sensors = sensors
        self.__car = car
        
    def rule_flag(self, traffic_lights):
        """
        Decides if the rule flag should be raised for this cycle (checks if the obstacle is a vehicle)
        \n\tINPUT(S):
        \n\tOUTPUT(S):
        """
        if(self.__sensors[0].get_other_actors() != []):
            if(self.__sensors[0].get_detections()[-1][0].type_id[:-2].startswith('vehicle')):
                return False
        return True
    
    def rule_follow(self, traffic_lights):
        """
        Decides how the car should follow this rule when it is triggered (tells the car to go)
        \n\tINPUT(S):
        \n\tOUTPUT(S):    
        """
        return 1