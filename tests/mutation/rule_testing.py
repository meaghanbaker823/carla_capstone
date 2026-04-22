import unittest
from unittest.mock import Mock
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from classes.rule import Rule

class RuleTesting(unittest.TestCase):
    def setUp(self):
        self.rule = Rule(Mock(), Mock())
        self.lights = Mock()
        self.limit = 30
    def test_rule_flag(self):
        self.assertEqual(self.rule.rule_flag(self.lights), None)

    def test_rule_follow(self):
        self.rule = Rule(Mock(), Mock())
        self.assertEqual(self.rule.rule_follow(self.lights, self.limit), None)

if __name__ == "__main__":
    unittest.main()