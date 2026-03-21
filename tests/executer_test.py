from unittest.mock import Mock
import unittest

# format_new_action test
class FormatNewActionTest(unittest.TestCase):
    def format_new_action(self):
        return ("The brake is: " + str(self.executer.get_new_action()["brake"]) +
        ", the throttle is: " + str(self.executer.get_new_action()["throttle"]) + 
        ", the steering is: " + str(self.executer.get_new_action()["steering"]))

    def test_1(self):
        # to interact with self. methods 
        self.executer = Mock()
        self.executer.get_new_action.return_value =  {"brake": 1, "steering": 0, "throttle": None}
        self.assertEqual(self.format_new_action(), "The brake is: 1, the throttle is: None, the steering is: 0" )

# execute test
class ExecuteTest(unittest.TestCase):
    def execute(self):
        self.new_action = {"brake": self.plan["brake"], "steering": self.plan["steering"], "throttle": self.plan["throttle"]}
                    
        self.car.apply_control(self.carla.VehicleControl(throttle = self.plan["throttle"] ,steer = self.plan["steering"], brake = self.plan["brake"]))

        return self.new_action
    
    def test_execute(self):
        self.car = Mock()
        self.carla = Mock()
        self.plan = {"brake": 1, "steering": 0, "throttle": None}

        self.assertEqual(self.execute(), self.plan)
# notify test

class NotifyTest(unittest.TestCase):
    def notify(self):
        return self.executer.format_new_action()
    
    def test_1(self):
        self.executer = Mock()
        self.executer.format_new_action.return_value = "The brake is: 1, the throttle is: None, the steering is: 0"

        self.assertEqual(self.notify(), "The brake is: 1, the throttle is: None, the steering is: 0")

if __name__ == "__main__":
    unittest.main()