from unittest.mock import Mock
from unittest.mock import patch
import unittest
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from classes.planner import Planner
    
# test plan
class PlannerTest(unittest.TestCase):
    def setUp(self):
        self.planner = Planner()
        self.observations = {"r": 0, "rules": [Mock(),], "traffic_lights": [Mock(), Mock()], "distance": 16, "route": [Mock(), Mock()],
                             "new_waypoint": 0, "steering_angle": 1, "current_speed": 32.4}
        self.DT = 0.005
        self.extra = 6
        self.u_nom = 1
        self.alpha = 1.5
        self.max_acc = 3
        self.max_brake = 6
        self.standard_dist = 8
        
    # tests steering == none branch and new_speed == None and new_speed > 0 branch
    @patch("classes.planner.CBFRule")
    def test_no_steer_and_throttle_zero(self, mock_cbf):
        self.observations["rules"][0].rule_follow.return_value = (None, None, None)

        cbf = mock_cbf.return_value
        cbf.calculate_min_distance.return_value = 8
        cbf.calculate_safety_function.return_value = 8
        cbf.calculate_allowable_distance.return_value = 0.6
        cbf.final_logic.return_value = 0.6

        with patch("builtins.print") as mock_print:
            self.assertDictEqual(self.planner.plan(self.observations, self.DT, self.extra, self.u_nom, self.alpha, self.max_acc, self.max_brake, self.standard_dist), {"brake": 0, "steering": 1, "throttle": 0.6})

    # tests steering != none branch and new_speed != None and new_speed <= 0 branch
    def test_steer_set_and_throttle_set(self):
        self.observations["rules"][0].rule_follow.return_value = (1, 0, -1)
        with patch("builtins.print") as mock_print:
            self.assertDictEqual(self.planner.plan(self.observations, self.DT, self.extra, self.u_nom, self.alpha, self.max_acc, self.max_brake, self.standard_dist), {"brake": 1, "steering": -1, "throttle": 0})
   

# test notify
class NotifyTest(unittest.TestCase):
    def setUp(self):
        self.planner = Planner()
        self.observations = {"r": 0, "rules": [Mock(),], "traffic_lights": [Mock(), Mock()], "distance": 16, "route": [Mock(), Mock()],
                             "new_waypoint": 0, "steering_angle": 1, "current_speed": 32.4}
        self.DT = 0.005
        self.extra = 6
        self.u_nom = 1
        self.alpha = 1.5
        self.max_acc = 3
        self.max_brake = 6
        self.standard_dist = 8

    def test_notify(self):
        self.observations["rules"][0].rule_follow.return_value = (1, 0, -1)
        with patch("builtins.print") as mock_print:
            self.planner.plan(self.observations, self.DT, self.extra, self.u_nom, self.alpha, self.max_acc, self.max_brake, self.standard_dist)
                                 
        self.assertEqual(self.planner.notify(), "The plan in this iteration is " + str({"brake": 1, "steering": -1, "throttle": 0}))


if __name__ == "__main__":
    unittest.main()