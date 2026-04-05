from classes.rule import Rule

class PedestrianRule(Rule):
    def __init__(self, sensors, car):
        super().__init__(sensors, car)
        self.__sensors = sensors
        self.__car = car

    def rule_flag(self, traffic_lights):
        if(self.__sensors[0].get_other_actors() != []):
            # print(self.__sensors[0].get_detections()[-1][0].type_id)
            if(self.__sensors[0].get_detections()[-1][0].type_id[:-2].startswith('walker')):
                return False
        return True

    def rule_follow(self, traffic_lights):
        # return brake as 1, speed as 0, and steering as None
        return 0
    
    """

class ObstacleRule(Rule):
    def __init__(self, sensors, car):
        super().__init__(sensors, car)
        self.__sensors = sensors
        self.__car = car

    def rule_flag(self, traffic_lights):
        if(self.__sensors[0].get_other_actors() != []):
            self.__car.avoid_obstacles()
            return False
        return True
    """