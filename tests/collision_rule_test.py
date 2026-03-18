from unittest.mock import Mock
import unittest
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from classes.collision_exception import CollisionErr


# rule_flag testing
class CollisionRuleTest(unittest.TestCase):
    def setUp(self):
        self.sensors = [Mock(), Mock()]
    
    def rule_flag(self):
        if(self.sensors[1].get_collisions() != []):
            raise CollisionErr("")
        return True
    
    def test_rule_flag1(self):
        # sensors1 is mimicing no collision detected
        self.sensors[1].get_collisions.return_value = []

        self.assertEqual(self.rule_flag(), True)    
    
    def test_rule_flag2(self):
        # sensor2 is mimicing a collision
        self.sensors[1].get_collisions.return_value = ["collision"]

        with self.assertRaises(CollisionErr):
            self.rule_flag()


# rule_follow testing
class CollisionRuleFollowTest(unittest.TestCase):
    def rule_follow(self):
        return 1, 0, None
    def test_rule_follow(self):
        self.assertEqual(self.rule_follow(),(1, 0, None) )




if __name__ == "__main__":
    unittest.main()