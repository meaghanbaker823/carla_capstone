from classes.rule import Rule

class CollisionRule(Rule):
    def __init__(self, sensors, car):
        super().__init__(sensors, car)
        self.__sensors = sensors
        self.__car = car
        
    def rule_flag(self, traffic_lights):
        global control_flag
        if(self.__sensors[1].get_collisions() != []):
            control_flag = False
            return False
        return True
    
    def rule_follow(self, traffic_lights, limit):
        # pass brake as 1 and speed as 0
        return 1, 0