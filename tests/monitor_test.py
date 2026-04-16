import unittest
from unittest.mock import Mock

import sys

sys.path.append("..")

from classes.world import World
from classes.trafficLight import TrafficLight
from classes.monitor import Monitor

import carla

def setUpModule():
    global monitor

    transform = [Mock(spec=carla.libcarla.Transform), Mock(spec=carla.libcarla.Transform)]

    global car

    car = Mock(spec=carla.libcarla.Vehicle)
    car.get_velocity.return_value = (0, 0, 0)

    blueprint = ["sensor.other.obstacle", "sensor.other.collision"]
    world = Mock(spec=World)
    world.get_actors.return_value = [str(car)]
    blueprint_lib = Mock()
    blueprint_lib.find.side_effect = ["sensor.other.obstacle", "sensor.other.collision"]
    actors = Mock()
    actors.filter.return_value = None
    global route
    route = [[Mock(spec=carla.libcarla.Waypoint)]] * 10

    for mock in route:
        mock[0].transform.location = None

    monitor = Monitor(transform, car, blueprint, world, blueprint_lib, actors, route)

class AdvanceWaypointTest(unittest.TestCase):
    def test_advance_waypoint_close(self):
        car.get_transform.return_value.location.distance.return_value = 3

        current_waypoint_num = 0

        result = monitor.advance_waypoint(current_waypoint_num)

        self.assertEqual(result, 9)

    def test_advance_waypoint_far(self):
        car.get_transform.return_value.location.distance.return_value = 6

        current_waypoint_num = 0

        result = monitor.advance_waypoint(current_waypoint_num)

        self.assertEqual(result, 0)


class GetLaneInfoTest(unittest.TestCase):
    def test_get_lane_info(self):
        waypoint = Mock(spec=carla.libcarla.Waypoint)

        waypoint.lane_id = -1
        waypoint.lane_type = "One Way"
        waypoint.lane_change = "Both"
        waypoint.left_lane_marking = "Dashed"
        waypoint.right_lane_marking = "Dashed"

        result = monitor.get_lane_info(waypoint)

        expectation = {"lane_id": -1, "lane_type": "One Way", "lane_change": "Both", "left_lane_marking": "Dashed", "right_lane_marking": "Dashed"}

        self.assertEqual(result, expectation)

class MonitorTest(unittest.TestCase):
    def test_monitor(self):
        current_waypoint_num = 0

        car.get_transform.return_value.location.distance.return_value = 6

        result = monitor.monitor(current_waypoint_num)

        self.assertEqual(result["current_velocity"], (0,0,0))
        self.assertIsInstance(result["traffic_lights"], TrafficLight)
        self.assertEqual(result["current_waypoint_num"], 0)
        self.assertEqual(result["route"], route)
        self.assertIsInstance(result["lane_info"], dict)

class NotifyTest(unittest.TestCase):
    def test_notify(self):
        result = monitor.notify()

        expectation = "The detections in this monitor iteration are: Detection: current_velocity Detection: traffic_lights Detection: current_waypoint_num Detection: route Detection: lane_info "

        self.assertEqual(result, expectation)

if __name__ == "__main__":
    unittest.main()
