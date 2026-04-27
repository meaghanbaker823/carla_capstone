import unittest
from unittest.mock import Mock

import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
from classes.monitor import Monitor

import carla

class TestMonitor(Monitor):
    def __init__(self, transform, car, blueprint, world, blueprint_lib, actors, route):
        self._Monitor__car = car
        self._Monitor__route = route

'''
This function calculates a metamorphic relation for the function advance_waypoint() in the Monitor

The relation is between the 
input: route, waypoint_idx
output: the advanced waypoint

This is done by returning the subset of the route which extends from the current waypoint index to
the end of the route.

advance_waypoint():
(waypoint_idx)           ->    (new_waypoint)

calculate_relation():
(route, waypoint_idx)    ->    (route_subset)
'''
def calculate_relation(route, waypoint_idx):
    return range(waypoint_idx, len(route))

class MetamorphicAdvanceWaypoint(unittest.TestCase):
    def setUp(self):
        self.car = Mock(spec=carla.libcarla.Vehicle)

        self.monitor = TestMonitor(None, self.car, None, None, None, None, [[Mock(spec=carla.libcarla.Waypoint)]] * 10)

    '''
    Each test case; labeled test_meta_0 - test_meta_3, test different scenarios by comparing the output
    of advance_waypoint() to the calculate_relation() function by verifying that the output index is
    within the subset of the route.
    '''
    def test_meta_0(self):
        self.car.get_transform.return_value.location.distance.return_value = 4

        waypoint_idx = 0

        result = self.monitor.advance_waypoint(waypoint_idx)
        
        expect = calculate_relation(self.monitor._Monitor__route, waypoint_idx)

        self.assertIn(result, expect)

    def test_meta_1(self):
        self.car.get_transform.return_value.location.distance.return_value = 4

        waypoint_idx = 5

        result = self.monitor.advance_waypoint(waypoint_idx)
        
        expect = calculate_relation(self.monitor._Monitor__route, waypoint_idx)

        self.assertIn(result, expect)

    def test_meta_2(self):
        self.car.get_transform.return_value.location.distance.return_value = 6

        waypoint_idx = 0

        result = self.monitor.advance_waypoint(waypoint_idx)
        
        expect = calculate_relation(self.monitor._Monitor__route, waypoint_idx)

        self.assertIn(result, expect)

    def test_meta_3(self):
        self.car.get_transform.return_value.location.distance.return_value = 3

        waypoint_idx = 5

        result = self.monitor.advance_waypoint(waypoint_idx)
        
        expect = calculate_relation(self.monitor._Monitor__route, waypoint_idx)

        self.assertIn(result, expect)

if __name__ == "__main__":
    unittest.main()
