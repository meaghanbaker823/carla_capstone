from unittest.mock import Mock
from unittest.mock import patch
import unittest
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
from executoradd5 import Executor

# format_new_action test
@patch("classes.executor.carla")
class FormatNewActionTest(unittest.TestCase):
    def test_1(self, mock_carla): 
        self.executer = Executor()
        self.plan = {"brake": 1, "steering": 0, "throttle": None}
        self.car = Mock()
        mock_carla.VehicleControl.return_value = Mock()
        with patch("builtins.print") as mock_print:
            self.executer.execute(self.plan, self.car)
        self.assertEqual(self.executer.format_new_action(), "The brake is: 1, the throttle is: None, the steering is: 0" )

# execute test
class ExecuteTest(unittest.TestCase): 
    @patch("classes.executor.carla")
    def test_execute(self, mock_carla):
        self.executer = Executor()
        self.plan = {"brake": 1, "steering": 0, "throttle": None}
        self.car = Mock()
        mock_carla.VehicleControl.return_value = Mock()

        with patch("builtins.print") as mock_print:
            self.assertEqual(self.executer.execute(self.plan, self.car), self.plan)
# notify test

class NotifyTest(unittest.TestCase):   
    @patch("classes.executor.carla")
    def test_1(self, mock_carla):
        self.executer = Executor()
        self.plan = {"brake": 1, "steering": 0, "throttle": None}
        self.car = Mock()
        mock_carla.VehicleControl.return_value = Mock()

        with patch("builtins.print") as mock_print:
            self.executer.execute(self.plan, self.car)
            self.assertEqual(self.executer.notify(), "The brake is: 1, the throttle is: None, the steering is: 0")

if __name__ == "__main__":
    unittest.main()