import unittest
from unittest.mock import Mock
from unittest.mock import patch
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from CBFRuleadd1 import CBFRule

# calc min dist testing
class CalcMinDistTesting(unittest.TestCase):
    def setUp(self):
        self.rule = CBFRule(0, 9, 16)
        self.extra = 6
        self.standard_dist = 8

    # testing the one execution path
    def test_case_one(self):
        self.assertEqual(self.rule.calculate_min_distance(self.extra, self.standard_dist), 8)

# calc safety func testing
class CalcSafetyFuncTesting(unittest.TestCase):
    def setUp(self):
        self.rule = CBFRule(0, 9, 16)
        self.min_dist = 8

    # testing the one execution path
    def test_case_one(self):
        self.assertEqual(self.rule.calculate_safety_function(self.min_dist), 8)

# calc allowable dist testing
class CalcAllowableDistTesting(unittest.TestCase):
    def setUp(self):
        self.rule = CBFRule(0, 9, 16)
        self.alpha = 1.5
        self.h = 8
        self.dt = 0.005

    # testing the one execution path
    def test_case_one(self):
        with patch("builtins.print") as mock_print:
            result = self.rule.calculate_allowable_distance(self.alpha, self.h, self.dt)
        self.assertEqual(result, 0.6)

# final logic testing
class FinalLogicTesting(unittest.TestCase):
    def setUp(self):
        self.rule = CBFRule(0, 9, 16)
        self.u_nom = 1
        self.u_cbf = 0.6
        self.max_acc = 3
        self.max_brake = 6

    # testing the one execution path
    def test_case_one(self):
        with patch("builtins.print") as mock_print:
            result = self.rule.final_logic(self.u_nom, self.u_cbf, self.max_acc, self.max_brake)
        self.assertEqual(result, 0.6)

# rule_flag testing
class RuleFlag(unittest.TestCase):
    def setUp(self):
        self.rule = CBFRule(0, 9, 16)
        self.lights = Mock()

    # testing the one execution path
    def test_case_one(self):
        self.assertEqual(self.rule.rule_flag(self.lights), True)
        

# rule_follow testing
class RuleFollow(unittest.TestCase):
    def setUp(self):
        self.rule = CBFRule(0, 9, 16)
        self.lights = Mock()
        self.limit = 30

    def test_rule_follow1(self):
        self.assertEqual(self.rule.rule_follow(self.lights, self.limit), (0, None, None))



if __name__ == "__main__":
    unittest.main()