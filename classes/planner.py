from classes.MAPEStep import MAPEStep
import traceback

class Planner(MAPEStep):
    def __init__(self):
        self.__old_parameters = []
        self.__new_parameters = []
        self.__old_plans = []
        self.__new_plan = {"brake": None, "steering": None, "throttle": None}

    def get_new_plan(self):
        return self.__new_plan
    
    def maintain_speed(self, current):
        try:
            assert current >= 0
        except:
            print(traceback.format_exc())

        new = 0

        for check in self.get_checks():
            if(check.speed_check(current, self.get_speed_limit()) != -1):
                new = check.speed_check(current, self.get_speed_limit())
                break
        
        try:
            assert 0 <= new <= 1
        except:
            print(traceback.format_exc())
        return new

    def plan(self, observations, limit):
        self.__old_plans.append(self.__new_plan)
        self.__new_plan = {"brake": None, "steering": None, "throttle": None}
        speed = observations["current_speed"]

        # rules, navigation, maintain speed
        for rule in observations["rules"]:
            self.__new_plan["brake"], speed = rule.rule_follow(observations["traffic_lights"], limit)
        
        self.__new_plan["steering"] = observations["steering_angle"]

        self.__new_plan["throttle"] = self.maintain_speed(speed)
        if (self.__new_plan["throttle"] == 0):
            self.__new_plan["brake"] = 1.0
        else:
            self.__new_plan["brake"] = 0.0
        
        return self.__new_plan
    
    def notify(self):
        return "The plan in this iteration is " + self.get_new_plan()