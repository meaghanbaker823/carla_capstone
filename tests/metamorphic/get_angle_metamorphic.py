import unittest
from unittest.mock import Mock

import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
from classes.analyzer import Analyzer

import carla
import math
import numpy

class Vec():
    def __init__(self, x, y):
        self.x = x
        self.y = y

'''
This function calculates a metamorphic relation for the function get_angle() in the Analyzer

The relation is between the:
input: (car_loc, car_fwd, wp_loc)
output: (degrees)

Because the get_angle() function employs the use of the angle_between() and correct_angle()
functions along with the AngleConstraint class these calculations were replication for determining
the relation.

get_angle():
(car, wp)                    ->    (corrected_deg)
(car_log, car_fwd, wp_loc)   ->    (degree)
'''
def calculate_relation(car_loc: Vec, car_fwd: Vec, wp_loc: Vec):
    denom = ((wp_loc.y - car_loc.y) ** 2 + (wp_loc.x - car_loc.x) **2) ** 0.5 
    x = (wp_loc.x - car_loc.x)/denom
    y = (wp_loc.y - car_loc.y)/denom

    n_vec = Vec(x, y)

    degrees = math.degrees(numpy.arctan2(n_vec.y, n_vec.x) - numpy.arctan2(car_fwd.y, car_fwd.x))

    if degrees < -300:
        degrees += 360
    elif degrees > 300:
        degrees -= 360

    if degrees < -50:
        degrees = -50
    elif degrees > 50:
        degrees = 50

    return degrees

class MetamorphicGetAngle(unittest.TestCase):
    def setUp(self):
        self.analyzer = Analyzer()

        self.car = Mock(spec=carla.libcarla.Vehicle)
        self.wp = Mock(spec=carla.libcarla.Waypoint)

    '''
    Each test case; labeled test_meta_0 - test_meta_5, test different scenarios by comparing the output
    of get_angle() to the output calculate_relation() function by verifying that the output is equal
    to the output of get_angle().
    '''
    def test_meta_0(self):
        car_loc = Vec(25, 30)
        car_fwd = Vec(0, 1)
        wp_loc = Vec(0, 0)

        self.car.get_transform.return_value.location.x = car_loc.x
        self.car.get_transform.return_value.location.y = car_loc.y

        self.wp.transform.location.x = wp_loc.x
        self.wp.transform.location.y = wp_loc.y

        self.car.get_transform.return_value.get_forward_vector.return_value = car_fwd

        result = self.analyzer.get_angle(self.car, self.wp)

        expect = calculate_relation(car_loc, car_fwd, wp_loc)

        self.assertEqual(result, expect)

    def test_meta_1(self):
        car_loc = Vec(25, 30)
        car_fwd = Vec(1, 0)
        wp_loc = Vec(0, 0)

        self.car.get_transform.return_value.location.x = car_loc.x
        self.car.get_transform.return_value.location.y = car_loc.y

        self.wp.transform.location.x = wp_loc.x
        self.wp.transform.location.y = wp_loc.y

        self.car.get_transform.return_value.get_forward_vector.return_value = car_fwd

        result = self.analyzer.get_angle(self.car, self.wp)

        expect = calculate_relation(car_loc, car_fwd, wp_loc)

        self.assertEqual(result, expect)

    def test_meta_2(self):
        car_loc = Vec(25, 30)
        car_fwd = Vec(0, -1)
        wp_loc = Vec(0, 0)

        self.car.get_transform.return_value.location.x = car_loc.x
        self.car.get_transform.return_value.location.y = car_loc.y

        self.wp.transform.location.x = wp_loc.x
        self.wp.transform.location.y = wp_loc.y

        self.car.get_transform.return_value.get_forward_vector.return_value = car_fwd

        result = self.analyzer.get_angle(self.car, self.wp)

        expect = calculate_relation(car_loc, car_fwd, wp_loc)

        self.assertEqual(result, expect)

    def test_meta_3(self):
        car_loc = Vec(25, 30)
        car_fwd = Vec(-1, 0)
        wp_loc = Vec(0, 0)

        self.car.get_transform.return_value.location.x = car_loc.x
        self.car.get_transform.return_value.location.y = car_loc.y

        self.wp.transform.location.x = wp_loc.x
        self.wp.transform.location.y = wp_loc.y

        self.car.get_transform.return_value.get_forward_vector.return_value = car_fwd

        result = self.analyzer.get_angle(self.car, self.wp)

        expect = calculate_relation(car_loc, car_fwd, wp_loc)

        self.assertEqual(result, expect)

    def test_meta_4(self):
        car_loc = Vec(0, 0)
        car_fwd = Vec(0, 1)
        wp_loc = Vec(25, 30)

        self.car.get_transform.return_value.location.x = car_loc.x
        self.car.get_transform.return_value.location.y = car_loc.y

        self.wp.transform.location.x = wp_loc.x
        self.wp.transform.location.y = wp_loc.y

        self.car.get_transform.return_value.get_forward_vector.return_value = car_fwd

        result = self.analyzer.get_angle(self.car, self.wp)

        expect = calculate_relation(car_loc, car_fwd, wp_loc)

        self.assertEqual(result, expect)

    def test_meta_5(self):
        car_loc = Vec(0, 0)
        car_fwd = Vec(1, 0)
        wp_loc = Vec(25, 30)

        self.car.get_transform.return_value.location.x = car_loc.x
        self.car.get_transform.return_value.location.y = car_loc.y

        self.wp.transform.location.x = wp_loc.x
        self.wp.transform.location.y = wp_loc.y

        self.car.get_transform.return_value.get_forward_vector.return_value = car_fwd

        result = self.analyzer.get_angle(self.car, self.wp)

        expect = calculate_relation(car_loc, car_fwd, wp_loc)

        self.assertEqual(result, expect)

    def test_meta_6(self):
        car_loc = Vec(0, 0)
        car_fwd = Vec(0, -1)
        wp_loc = Vec(25, 30)

        self.car.get_transform.return_value.location.x = car_loc.x
        self.car.get_transform.return_value.location.y = car_loc.y

        self.wp.transform.location.x = wp_loc.x
        self.wp.transform.location.y = wp_loc.y

        self.car.get_transform.return_value.get_forward_vector.return_value = car_fwd

        result = self.analyzer.get_angle(self.car, self.wp)

        expect = calculate_relation(car_loc, car_fwd, wp_loc)

        self.assertEqual(result, expect)

    def test_meta_7(self):
        car_loc = Vec(0, 0)
        car_fwd = Vec(-1, 0)
        wp_loc = Vec(25, 30)

        self.car.get_transform.return_value.location.x = car_loc.x
        self.car.get_transform.return_value.location.y = car_loc.y

        self.wp.transform.location.x = wp_loc.x
        self.wp.transform.location.y = wp_loc.y

        self.car.get_transform.return_value.get_forward_vector.return_value = car_fwd

        result = self.analyzer.get_angle(self.car, self.wp)

        expect = calculate_relation(car_loc, car_fwd, wp_loc)

        self.assertEqual(result, expect)

if __name__ == "__main__":
    unittest.main()
