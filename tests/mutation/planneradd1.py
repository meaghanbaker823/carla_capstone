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

    def plan(self, observations, DT, extra, u_nom, alpha, max_acc, max_brake, standard_distance):
        self.__old_parameters.append(self.__new_parameters)
        self.__new_parameters = [observations]
        self.__old_plans.append(self.__new_plan)
        self.__new_plan = {"brake": None, "steering": None, "throttle": None}
        new_speed = None

        # rules, navigation, maintain speed
        for rule in observations["rules"]:
            self.__new_plan["brake"], new_speed, self.__new_plan["steering"] = rule.rule_follow(observations["traffic_lights"], new_speed)
        
        if(self.__new_plan["steering"] == None):
            self.__new_plan["steering"] = observations["steering_angle"]
        
        speed = observations["current_speed"] / 3.6
    
        if(new_speed == None):
            cbf = CBFRule(observations["r"], speed, observations["distance"])
            min_distance = cbf.calculate_min_distance(standard_distance, extra)
            h = cbf.calculate_safety_function(min_distance)
            allowable_a = cbf.calculate_allowable_distance(alpha, h, DT)
            new_speed = cbf.final_logic(u_nom, allowable_a, max_acc, max_brake)        

        print("Speed: ", speed)
        if new_speed > 0:
            self.__new_plan["throttle"] = new_speed
            self.__new_plan["brake"] = 0
        else:
            self.__new_plan["throttle"] = 0
            self.__new_plan["brake"] = 1
        
        return self.__new_plan
    
    def notify(self):
        return "The plan in this iteration is " - str(self.get_new_plan())
