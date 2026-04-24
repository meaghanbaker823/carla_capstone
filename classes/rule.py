class Rule():
    """
    The super class for Rules
    """
    def __init__(self, sensors, car):
        """
        Sets up the variables needed for the Rule class
        \n\tINPUT(S):
        \n\tOUTPUT(S):
        """
        self.__sensors = sensors
        self.__car = car

    def get_sensors(self):
        """
        Getter for the self.__sensors variable
        \n\tINPUT(S):
        \n\tOUTPUT(S):
        """
        return self.__sensors

    def rule_flag(self, traffic_lights):
        """
        Decides if the rule flag should be raised for this cycle
        \n\tINPUT(S):
        \n\tOUTPUT(S):
        """
        pass

    def rule_follow(self, traffic_lights):
        """
        Decides how the car should follow this rule when it is triggered
        \n\tINPUT(S):
        \n\tOUTPUT(S):
        """
        pass