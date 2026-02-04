from classes.rule import Rule

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