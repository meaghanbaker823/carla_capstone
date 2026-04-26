from classes.rule import Rule

class PedestrianRule(Rule):
    """
    The rule subclass for when a car's obstacle detector is triggered from a pedestrian
    """
    def __init__(self, sensors, car):
        """
        Sets up the variables needed for the PedestrianRule class
        \n\tINPUT(S):sensors: a list of the sensors attached to the vehicle,
                      car: the vehicle object
        \n\tOUTPUT(S): N/A
        """
        super().__init__(sensors, car)
        self.__sensors = sensors
        self.__car = car

    def rule_flag(self, traffic_lights):
        """
        Decides if the rule flag should be raised for this cycle (Checks if sensor detected a walker)
        \n\tINPUT(S): traffic_lights: a list of the traffic lights on the map
        \n\tOUTPUT(S): Returns False if there is a pedestrian, otherwise True
        """
        if(self.__sensors[0].get_other_actors() != []):
            if(self.__sensors[0].get_detections()[-1][0].type_id[:-2].startswith('walker')):
                return False
        return True

    def rule_follow(self, traffic_lights):
       """
        Decides how the car should follow this rule when it is triggered (Will tell the car to stop)
        \n\tINPUT(S): traffic_lights: a list of the traffic lights on the map
        \n\tOUTPUT(S): returns 0 
        """
       return 0