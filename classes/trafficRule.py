from classes.rule import Rule

class TrafficRule(Rule):
    def __init__(self, sensors, car):
        super().__init__(sensors, car)
        self.__sensors = sensors
        self.__car = car

    def rule_flag(self, traffic_lights):
        if(traffic_lights.process_color(self.__car.get_car()) != ""):
            if(traffic_lights.get_response() == "stop"):
                self.__car.stop_car()
            else:
                self.__car.drive()
            return False
            
        return True