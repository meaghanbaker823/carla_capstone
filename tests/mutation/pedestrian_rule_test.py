import unittest
from unittest.mock import Mock
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from classes.pedestrianRule import PedestrianRule

class RuleFlagTest(unittest.TestCase):
    def setUp(self):
        self.sensors = [Mock(), Mock()]
        self.car = Mock()
        self.rule = PedestrianRule(self.sensors, self.car)
        self.limit = 30
        self.lights = Mock()
        self.detections = [[Mock(),],]
        self.sensors[0].get_detections.return_value = self.detections
    
    # other actors not empty, starts with vehicle -> returns True
    def test_case_one(self):
        self.detections[-1][0].type_id = "vehicle.ford.mustang"
        self.assertEqual(self.rule.rule_flag(self.lights), True)

    # other actors not empty, not a vehicle -> returns False
    def test_case_two(self):
        self.detections[-1][0].type_id = "walker.pedestrian.0001"
        self.sensors[0].get_other_actors.return_value = [Mock(),]
        self.assertEqual(self.rule.rule_flag(self.lights), False)

    # other actors empty, returns True
    def test_case_three(self):
        self.sensors[0].get_other_actors.return_value = []
        self.assertEqual(self.rule.rule_flag(self.lights), True)

    
# rule_follow testing
class RuleFollow(unittest.TestCase):
    def setUp(self):
        self.sensors = [Mock(), Mock()]
        self.car = Mock()
        self.rule = PedestrianRule(self.sensors, self.car)
        self.lights = Mock()
        self.limit = 30

    def test_rule_follow1(self):
        self.assertEqual(self.rule.rule_follow(self.lights, self.limit), (1, 0, None))


if __name__ == "__main__":
    unittest.main()