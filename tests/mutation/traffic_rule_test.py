import unittest
from unittest.mock import Mock
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from trafficRuleequals1 import TrafficRule

# rule_flag testing
class RuleFlag(unittest.TestCase):    
    def setUp(self):
        self.sensors = [Mock(), Mock()]
        self.car = Mock()
        self.rule = TrafficRule(self.sensors, self.car)
        self.lights = Mock()
        self.limit = 30


    def test_rule_flag1(self):
        # lights 1 simulating no response needed from car
        self.lights.process_color.return_value = ""
        self.assertEqual(self.rule.rule_flag(self.lights), False)


    def test_rule_flag2(self):
        # light2 simulating a response needed from car
        self.lights.process_color.return_value = "red"
        self.assertEqual(self.rule.rule_flag(self.lights), False)
        



# rule_follow testing
class RuleFollow(unittest.TestCase):
    def setUp(self):
        self.sensors = [Mock(), Mock()]
        self.car = Mock()
        self.rule = TrafficRule(self.sensors, self.car)
        self.lights = Mock()
        self.limit = 30
    
    def test_rule_follow1(self):
        # lights1 mimicing case where continue driving 
        self.lights.get_response.return_value = "drive"
        self.assertEqual(self.rule.rule_follow(self.lights, self.limit), (0, 30, None))

    def test_rule_follow2(self):
        # lights2 mimicing case where need to stop car
        self.lights.get_response.return_value = "stop"
        self.assertEqual(self.rule.rule_follow(self.lights, self.limit), (1, 0, None))





if __name__ == "__main__":
    unittest.main()