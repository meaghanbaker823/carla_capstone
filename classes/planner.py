from classes.MAPEStep import MAPEStep

"""
===========
Planner Class()

__init__(self) creates instance and initilizes attributes
    self.__

function(self) return type | description

===========
"""

class Planner(MAPEStep):
    def __init__(self):
        self.__old_parameters = []
        self.__new_parameters = []
        self.__old_plans = []
        self.__new_plan = {"brake": None, "steering": None, "throttle": None}

    def get_new_plan(self):
        return self.__new_plan
    
    def maintain_speed(self, current, limit, checks):
        try:
            assert current >= 0
        except:
            raise

        new = 0

        for check in checks:
            if(check.speed_check(current, limit) != -1):
                new = check.speed_check(current, limit)
                break
        
        try:
            assert 0 <= new <= 1
        except:
            raise
        return new

    def plan(self, observations, limit, checks):
        self.__old_parameters.append(self.__new_parameters)
        self.__new_parameters = [observations, limit, checks]
        self.__old_plans.append(self.__new_plan)
        self.__new_plan = {"brake": None, "steering": None, "throttle": None}
        speed = observations["current_speed"]

        # rules, navigation, maintain speed
        for rule in observations["rules"]:
            self.__new_plan["brake"], limit = rule.rule_follow(observations["traffic_lights"], limit)
        
        self.__new_plan["steering"] = observations["steering_angle"]

        self.__new_plan["throttle"] = self.maintain_speed(speed, limit, checks)
        if (self.__new_plan["throttle"] == 0):
            self.__new_plan["brake"] = 1.0
        else:
            self.__new_plan["brake"] = 0.0
        
        return self.__new_plan
    
    def notify(self):
        return "The plan in this iteration is " + self.get_new_plan()