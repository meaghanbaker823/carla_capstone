import unittest
from unittest.mock import Mock
from unittest.mock import patch

import sys

sys.path.append("..")

import classes
from classes.parkedRule import ParkedRule
from classes.routeDone import RouteDone
from classes.analyzer import Analyzer

import carla

def setUpModule():
    global analyzer
    analyzer = Analyzer()

class AngleBetweenTest(unittest.TestCase):
    def test_angle_between(self):
        v1 = (0, 1)
        v2 = (1, 0)

        result = analyzer.angle_between(v1, v2)

        self.assertEqual(result, 90)

class GetAngleTest(unittest.TestCase):
    def test_get_angle(self):
        car = Mock(spec=carla.libcarla.Vehicle)
        waypoint = Mock(spec=carla.libcarla.Waypoint)

        car_transform = Mock(spec=carla.libcarla.Transform)

        car.get_transform.return_value = car_transform

        car_transform.location.x = 25
        car_transform.location.y = 30

        waypoint.transform.location.x = 0
        waypoint.transform.location.y = 0

        car_vector = Mock()
        car_vector.x = 0
        car_vector.y = 1

        car_transform.get_forward_vector.return_value = car_vector

        result = analyzer.get_angle(car, waypoint)

        self.assertEqual(result, -50)

class GetProperAngleTest(unittest.TestCase):
    def test_get_proper_angle(self):
        car = Mock(spec=carla.libcarla.Vehicle)
        waypoint_num = 0
        route = [[Mock(carla.libcarla.Waypoint)]] * 10

        for i in range(len(route)):
            route[i][0].transform.location.x = 0
            route[i][0].transform.location.y = 0

        car_transform = Mock(spec=carla.libcarla.Transform)

        car_transform.location.x = 25
        car_transform.location.y = 30

        car_vector = Mock()
        car_vector.x = 0
        car_vector.y = 1

        car_transform.get_forward_vector.return_value = car_vector

        car.get_transform.return_value = car_transform

        result = analyzer.get_proper_angle(car, waypoint_num, route)

        self.assertEqual(result, (3, -50))

    def test_get_proper_angle_end(self):
        car = Mock(spec=carla.libcarla.Vehicle)
        waypoint_num = 9
        route = [[Mock(carla.libcarla.Waypoint)]] * 10

        for i in range(len(route)):
            route[i][0].transform.location.x = 0
            route[i][0].transform.location.y = 0

        car_transform = Mock(spec=carla.libcarla.Transform)

        car_transform.location.x = 25
        car_transform.location.y = 30

        car_vector = Mock()
        car_vector.x = 0
        car_vector.y = 1

        car_transform.get_forward_vector.return_value = car_vector

        car.get_transform.return_value = car_transform

        with self.assertRaises(RouteDone):
            analyzer.get_proper_angle(car, waypoint_num, route)

class CorrectAngleTest(unittest.TestCase):
    def test_correct_angle_minimum(self):
        degrees = -310

        result = analyzer.correct_angle(degrees)

        self.assertEqual(result, 50)

    def test_correct_angle_maximum(self):
        degrees = 310

        result = analyzer.correct_angle(degrees)

        self.assertEqual(result, -50)

    def test_correct_anlge_between(self):
        degrees = 10

        result = analyzer.correct_angle(degrees)

        self.assertEqual(result, 10)

class CheckLaneOptionTest(unittest.TestCase):
    def test_check_lane_options_none(self):
        waypoint_num = 0
        route = [[Mock(spec=carla.libcarla.Waypoint)]]
        lane_change = carla.libcarla.LaneChange.NONE

        result = analyzer.check_lane_options(waypoint_num, route, lane_change)

        self.assertEqual(result, None)
    
    def test_check_lane_options_right(self):
        waypoint_num = 0
        route = [Mock(spec=carla.libcarla.Waypoint)]
        lane_change = carla.libcarla.LaneChange.Right

        right_lane = Mock()
        right_lane.next.return_value = ["success"]

        route[0].get_right_lane.return_value = right_lane

        result = analyzer.check_lane_options(waypoint_num, route, lane_change)

        self.assertEqual(result, "success")

    def test_check_lane_options_both(self):
        waypoint_num = 0
        route = [Mock(spec=carla.libcarla.Waypoint)]
        lane_change = carla.libcarla.LaneChange.Both

        right_lane = Mock()
        right_lane.next.return_value = ["success"]

        route[0].get_right_lane.return_value = right_lane

        result = analyzer.check_lane_options(waypoint_num, route, lane_change)

        self.assertEqual(result, "success")

    def test_check_lane_options_left(self):
        waypoint_num = 0
        route = [Mock(spec=carla.libcarla.Waypoint)]
        lane_change = carla.libcarla.LaneChange.Left

        left_lane = Mock()
        left_lane.previous.return_value = ["success"]

        route[0].get_left_lane.return_value = left_lane

        result = analyzer.check_lane_options(waypoint_num, route, lane_change)

        self.assertEqual(result, "success")
class AnalyzeTest(unittest.TestCase):
    @patch.object(Analyzer, "get_proper_angle")
    @patch.object(Analyzer, "check_lane_options")
    def test_analzye(self, mock_check_lane_options, mock_get_proper_angle):
        car = Mock(spec=carla.libcarla.Vehicle)
        rules = [Mock(spec=ParkedRule), Mock()]
        rules[0].rule_flag.return_value = True
        rules[1].rule_flag.return_value = False

        rule_sensor = Mock()
        rule_sensor.get_detections = [[1, 1]]

        rules[0].get_sensors.return_value = [rule_sensor]

        detections = {}
        detections["traffic_lights"] = None
        detections["current_waypoint_num"] = 0
        detections["route"] = [Mock(spec=carla.libcarla.Waypoint)]
        detections["lane_info"] = {"lane_change": carla.libcarla.LaneChange.NONE}
        current_velocity = Mock()
        current_velocity.x = 1
        current_velocity.y = 1
        current_velocity.z = 1
        detections["current_velocity"] = current_velocity

        mock_check_lane_options.return_value = True
        mock_get_proper_angle.return_value = (0, 0)

        distance = 0 

        result = analyzer.analyze(car, rules, detections, distance)

        self.assertEqual(result["r"], 0)
        self.assertEqual(result["rules"][0], rules[1])
        self.assertEqual(result["traffic_lights"], None)
        self.assertEqual(result["distance"], 0)
        self.assertEqual(result["open_lane"], True)
        self.assertEqual(result["current_speed"], 6)
        self.assertEqual(result["new_waypoint"], 0)
        self.assertEqual(result["steering_angle"], 0)

class NotifyTest(unittest.TestCase):
    def test_notify(self):
        result = analyzer.notify()

        expectation = "The observations in this analyzer iteration are: Observation: r Observation: rules Observation: traffic_lights Observation: distance Observation: open_lane Observation: current_speed Observation: new_waypoint Observation: steering_angle "
        self.assertEqual(result, expectation)

if __name__ == "__main__":
    unittest.main()
