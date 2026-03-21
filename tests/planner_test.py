from unittest.mock import Mock
import unittest

# test maintain_speed
class MaintainSpeedTesting(unittest.TestCase):
    def setUp(self):
        #checks = [SpeedOverCheck(speed_threshold), SpeedPerfectCheck(speed_threshold), SpeedUnderCheck(speed_threshold)]
        self.checks = [Mock(), Mock(), Mock()]
        self.limit = 30
        self.new = 0
        self.threshold = 3

    
    def maintain_speed(self):
        new = 0
        for check in self.checks:
            if(check.speed_check(self.current, self.limit) != -1):
                new = check.speed_check(self.current, self.limit)
                break
        
        return new
    
    def test_maintain_speed1(self):
        # none of the checks return -1
        self.current = 28

        # setting up checks
        if(self.current >= self.limit):
            self.checks[0].speed_check.return_value =  0
        else:
            self.checks[0].speed_check.return_value =  -1

        if(self.current == (self.limit - self.threshold)):
            self.checks[1].speed_check.return_value  = 0.3
        else:
            self.checks[1].speed_check.return_value  = -1

        if(self.current < (self.limit - self.threshold)):
            self.checks[2].speed_check.return_value =  0.8
        else:
            self.checks[2].speed_check.return_value =  -1

        self.assertEqual(self.maintain_speed(), 0)

    def test_maintain_speed2(self):
        # need to change speed brach
        self.current = 26

        # setting up checks
        if(self.current >= self.limit):
            self.checks[0].speed_check.return_value =  0
        else:
            self.checks[0].speed_check.return_value =  -1

        if(self.current == (self.limit - self.threshold)):
            self.checks[1].speed_check.return_value  = 0.3
        else:
            self.checks[1].speed_check.return_value  = -1

        if(self.current < (self.limit - self.threshold)):
            self.checks[2].speed_check.return_value =  0.8
        else:
            self.checks[2].speed_check.return_value =  -1

        self.assertEqual(self.maintain_speed(), 0.8)
    
# test plan
class PlannerTest(unittest.TestCase):
    def setUp(self):
        self.speed = 30
        self.new_plan = {"brake": None, "steering": None, "throttle": None}
        self.observations = {"rules": [Mock(),], "steering_angle": 1, "distance": 20, "traffic_lights": Mock()}
        self.checks = [Mock(), Mock(), Mock()]
        self.limit = 30

        # to interact with self. methods
        self.planner = Mock()
        
    
    def plan(self):
        # rules, navigation, maintain speed
        for rule in self.observations["rules"]:
            self.new_plan["brake"], self.limit, self.new_plan["steering"] = rule.rule_follow(self.observations["traffic_lights"], self.limit)
        
        if(self.new_plan["steering"] == None):
            self.new_plan["steering"] = self.observations["steering_angle"]

        self.new_plan["throttle"] = self.planner.maintain_speed(self.speed, self.limit, self.checks)
        if (self.new_plan["throttle"] == 0):
            self.new_plan["brake"] = 1.0
        else:
            self.new_plan["brake"] = 0.0
        
        return self.new_plan
    
    def test_no_steer_and_throttle_zero(self):
        self.observations["rules"][0].rule_follow.return_value = (1, 0, None)
        self.planner.maintain_speed.return_value = 0

        self.assertDictEqual(self.plan(), {"brake": 1, "steering": 1, "throttle": 0})

    def test_steer_set_and_throttle_set(self):
        self.observations["rules"][0].rule_follow.return_value = (0, self.limit, -1)
        self.planner.maintain_speed.return_value = 0.8

        self.assertDictEqual(self.plan(), {"brake": 0, "steering": -1, "throttle": 0.8})
   

# test notify
class NotifyTest(unittest.TestCase):
    def notify(self):
        return "The plan in this iteration is " + str(self.plan.get_new_plan())
    
    def test_notify(self):
        self.plan = Mock()
        self.plan.get_new_plan.return_value = {"brake": 1, "steering": 0, "throttle": None}

        self.assertEqual(self.notify(), "The plan in this iteration is " + str({"brake": 1, "steering": 0, "throttle": None}))


if __name__ == "__main__":
    unittest.main()