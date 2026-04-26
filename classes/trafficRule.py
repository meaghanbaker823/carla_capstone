from classes.rule import Rule

class TrafficRule(Rule):
    """
    The rule subclass for when a car's traffic light detection is triggered
    """
    def __init__(self, sensors, car):
        """
        Sets up the variables needed for the TrafficRule class
        \n\tINPUT(S): sensors: a list of the sensors attached to the vehicle,
                      car: the vehicle object
        \n\tOUTPUT(S): N/A
        """
        super().__init__(sensors, car)
        self.__sensors = sensors
        self.__car = car

    def rule_flag(self, traffic_lights):
        """
        Decides if the rule flag should be raised for this cycle (checks if the car's process color is empty)
        \n\tINPUT(S): traffic_lights: a list of the traffic lights on the map
        \n\tOUTPUT(S): Returns False if there is a light to react to, otherwise True
        """
        if(traffic_lights.process_color(self.__car.get_car()) != ""):
            return False
        return True
    
    def rule_follow(self, traffic_lights):
        """
        Decides how the car should follow this rule when it is triggered (Will tell the car to stop if response is stop, else go)
        \n\tINPUT(S): traffic_lights: a list of the traffic lights on the map
        \n\tOUTPUT(S): returns 1 if need to stop, otherwise 1  
        """
        if(traffic_lights.get_response() == "stop"):
            return 0
        else:
            return 1