class Rule():
    """
    The super class for Rules
    """
    def __init__(self, sensors, car):
        """
        Sets up the variables needed for the Rule class
        \n\tINPUT(S): sensors: a list of the sensors on the vehicle,
                      car: the carla vehicle
        \n\tOUTPUT(S): N/A
        """
        self.__sensors = sensors
        self.__car = car

    def get_sensors(self):
        """
        Getter for the self.__sensors variable
        \n\tINPUT(S): N/A
        \n\tOUTPUT(S): the list of sensors attached to the vehicle
        """
        return self.__sensors

    def rule_flag(self, traffic_lights):
        """
        Decides if the rule flag should be raised for this cycle
        \n\tINPUT(S): traffic_lights: a list of the traffic lights on the map
        \n\tOUTPUT(S): N/A
        """
        pass

    def rule_follow(self, traffic_lights):
        """
        Decides how the car should follow this rule when it is triggered
        \n\tINPUT(S): traffic_lights: a list of the traffic lights on the map
        \n\tOUTPUT(S): N/A
        """
        pass