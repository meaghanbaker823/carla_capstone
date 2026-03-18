import unittest
from unittest.mock import Mock
# rule_flag testing
class RuleFlag(unittest.TestCase):
    def rule_flag(self):
        if self.lights.process_color.return_value != "":
            return False
        return True
    
    def setUp(self):
        self.lights = Mock()


    def test_rule_flag1(self):
        # lights 1 simulating no response needed from car
        self.lights.process_color.return_value = ""
        return_value = self.rule_flag()

        self.assertEqual(return_value, True)


    def test_rule_flag2(self):
        # light2 simulating a response needed from car
        self.lights.process_color.return_value = "red"
        return_value = self.rule_flag()
        self.assertEqual(return_value, False)
        



# rule_follow testing
class RuleFollow(unittest.TestCase):
    def setUp(self):
        self.lights = Mock()
        # setting limit to standard speed limit
        self.limit = 30
    
    def rule_follow(self):
        if(self.lights.get_response() == "stop"):
            return 1, 0, None
        else:
            return 0, self.limit, None
        
    def test_rule_follow1(self):
        # lights1 mimicing case where continue driving 
        self.lights.get_response.return_value = "drive"

        self.assertEqual(self.rule_follow(), (0, 30, None))

    def test_rule_follow2(self):
        # lights2 mimicing case where need to stop car
        self.lights.get_response.return_value = "stop"
        self.assertEqual(self.rule_follow(), (1, 0, None))





if __name__ == "__main__":
    unittest.main()