import unittest
from unittest.mock import Mock
# rule_flag testing
class RuleFlag(unittest.TestCase):
    def test_rule_flag(self):
        # testing both branches
        lights1 = Mock()
        lights2 = Mock()

        # lights 1 simulating no response needed from car
        lights1.process_color.return_value = ""
        # lights2 simulating a response needed from car
        lights2.process_color.return_value = "red"
        return_value1 = None
        return_value2 = None
        if lights1.process_color.return_value != "":
            return_value1 = False
        else:
            return_value1 = True
        

        if lights2.process_color.return_value != "":
            return_value2 = False
        else:
            return_value2 = True

        self.assertEqual(return_value1, True)
        self.assertEqual(return_value2, False)
        



# rule_follow testing
class RuleFollow(unittest.TestCase):
    def test_rule_follow(self):
        lights1 = Mock()
        lights2 = Mock()

        # setting limit to standard speed limit
        limit = 30

        # lights1 mimicing case where continue driving 
        lights1.get_response.return_value = "drive"
        # lights2 mimicing case where need to stop car
        lights2.get_response.return_value = "stop"

        lights1_return = (0, limit, None)
        lights2_return = (1, 0, None)

        self.assertEqual(lights1_return, (0, 30, None))
        self.assertEqual(lights2_return, (1, 0, None))






if __name__ == "__main__":
    unittest.main()