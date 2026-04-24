from unittest.mock import Mock
import unittest
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from classes.collisionException import CollisionErr
from classes.collisionRule import CollisionRule


# rule_flag testing
class CollisionRuleTest(unittest.TestCase):
    def setUp(self):
        self.sensors = [Mock(), Mock()]
        self.car = Mock()
        self.rule = CollisionRule(self.sensors, self.car)
        self.lights = Mock()
        self.limit = 30
    
    def test_rule_flag1(self):
        # sensors1 is mimicing no collision detected
        self.sensors[1].get_collisions.return_value = []

        self.assertEqual(self.rule.rule_flag(self.lights), True)    
    
    def test_rule_flag2(self):
        # sensor2 is mimicing a collision
        self.sensors[1].get_collisions.return_value = ["collision",]

        with self.assertRaises(CollisionErr):
            self.rule.rule_flag(self.lights)


# rule_follow testing
class CollisionRuleFollowTest(unittest.TestCase):
    def setUp(self):
        self.rule = CollisionRule(Mock(), Mock())
        self.lights = Mock()
        self.limit = 30
    def test_rule_follow(self):
        self.assertEqual(self.rule.rule_follow(self.lights, self.limit),(1, 0, None) )




if __name__ == "__main__":
    unittest.main()