from classes.rule import Rule

class ParkedRule(Rule):
    """
    The rule subclass for when a car's obstacle detector is triggered by a car
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
        Decides if the rule flag should be raised for this cycle 
        (checks if the obstacle is a vehicle)
        \n\tINPUT(S): traffic_lights: a list of the traffic lights on the map
        \n\tOUTPUT(S): Returns False if there is a parked car, otherwise True
        """
        if(self.__sensors[0].get_other_actors() != []):
            if(self.__sensors[0].get_detections()[-1][0].type_id[:-2].startswith('vehicle')):
                return False
        return True
    
    def rule_follow(self, traffic_lights):
        """
        Decides how the car should follow this rule when it is triggered (tells the car to go)
        \n\tINPUT(S): traffic_lights: a list of the traffic lights on the map
        \n\tOUTPUT(S): returns 1  
        """
        return 1