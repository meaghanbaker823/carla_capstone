from classes.rule import Rule

class TrafficRule(Rule):
    def __init__(self, sensors, car):
        super().__init__(sensors, car)
        self.__sensors = sensors
        self.__car = car

    def rule_flag(self, traffic_lights):
        if(traffic_lights.process_color(self.__car.get_car()) != ""):
            return False
            
        return True
    
    def rule_follow(self, traffic_lights, limit):
        if(traffic_lights.get_response() == "stop"):
            return 1, 0
        else:
            return 0, limit