import unittest
from unittest.mock import Mock
# rule_flag testing
class RuleFlag(unittest.TestCase):
    def rule_flag(self, traffic_lights):
        if(self.sensors[0].get_other_actors() != []):
            if(self.sensors[0].get_detections()[-1][0].type_id[:-2].startswith('vehicle')):
                return False
        return True
    
    def setUp(self):
        self.lights = Mock()
        self.sensors = [Mock(),]
        self.lights = Mock()

    # other actors not empty, starts with vehicle -> returns False
    def test_case_one(self):
        self.assertEqual(self.rule_flag(self.lights), False)
    # other actors not empty, not a vehicle -> returns True
    
    def test_case_two(self):
        self.sensors[0].get_other_actors.return_value = [Mock(),]
        self.assertEqual(self.rule_flag(self.lights), True)

    # other actors empty, returns True
    def test_case_three(self):
        self.sensors[0].get_other_actors.return_value = []
        self.assertEqual(self.rule_flag(self.lights), True)
        

# rule_follow testing
class RuleFollow(unittest.TestCase):
    def rule_follow(self):
        return None, None, None
        
    def test_rule_follow1(self):
        self.assertEqual(self.rule_follow(), (None, None, None))

    





if __name__ == "__main__":
    unittest.main()