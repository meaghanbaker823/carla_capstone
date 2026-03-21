from classes.MAPEStep import MAPEStep
from classes.CBFRule import CBFRule

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
        observations["distance"] = 20

        # rules, navigation, maintain speed
        for rule in observations["rules"]:
            self.__new_plan["brake"], limit, self.__new_plan["steering"] = rule.rule_follow(observations["traffic_lights"], limit)
        
        cbf = CBFRule(observations["r"], observations["distance"], observations["current_speed"])
        min_distance = cbf.calculate_min_distance(10)
        h = cbf.calculate_safety_function(min_distance)
        allowable_a = cbf.calculate_allowable_distance(1, h, 0.005)
        u = cbf.final_logic(cbf.get_u_nom(), allowable_a, 1)

        if(self.__new_plan["steering"] == None):
            self.__new_plan["steering"] = observations["steering_angle"]

        self.__new_plan["throttle"] = self.maintain_speed(speed, limit, checks)
        if (self.__new_plan["throttle"] == 0):
            self.__new_plan["brake"] = 1.0
        else:
            self.__new_plan["brake"] = 0.0
        
        return self.__new_plan
    
    def notify(self):
        return "The plan in this iteration is " + str(self.get_new_plan())
