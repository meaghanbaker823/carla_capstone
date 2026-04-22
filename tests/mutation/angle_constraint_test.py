import unittest

import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from angleConstraintadd1 import AngleConstraint

def setUpModule():
    global angle_constraint
    global minimum
    global maximum
    global adjust

    minimum = -360
    maximum = 360
    adjust = 30

    angle_constraint = AngleConstraint(minimum, maximum, adjust)

class ClampTest(unittest.TestCase):
    def test_clamp_less_than(self):
        less_degree = -370

        result = angle_constraint.clamp(less_degree)

        self.assertEqual(result, less_degree + adjust)

    def test_clamp_greater_than(self):
        greater_degree = 370

        result = angle_constraint.clamp(greater_degree)

        self.assertEqual(result, greater_degree - adjust)

    def test_clamp_in_bounds(self):
        bounds_degree = 0

        result = angle_constraint.clamp(bounds_degree)

        self.assertEqual(result, bounds_degree)

class MaxSteerTest(unittest.TestCase):
    def test_max_steer_less_than(self):
        less_degree = -370

        result = angle_constraint.max_steer(less_degree)

        self.assertEqual(result, minimum)

    def test_max_steer_greater_than(self):
        greater_degree = 370

        result = angle_constraint.max_steer(greater_degree)

        self.assertEqual(result, maximum)

    def test_max_steer_in_bounds(self):
        bounds_degree = 0

        result = angle_constraint.max_steer(bounds_degree)

        self.assertEqual(result, bounds_degree)

if __name__ == "__main__":
    unittest.main()
