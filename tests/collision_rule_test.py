from unittest.mock import Mock
import unittest


# rule_flag testing
class CollisionRuleTest(unittest.TestCase):
    def test_rule_flag(self):
        sensors1 = Mock()
        sensors2 = Mock()

        # sensors1 is mimicing no collision detected
        sensors1.get_collisions().return_value = []
        # sensor2 is mimicing a collision
        sensors2.get_collisions().return_value = ["collision"]

        

# rule_follow testing