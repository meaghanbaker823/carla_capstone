import unittest
from unittest.mock import Mock
from unittest.mock import MagicMock
from unittest.mock import patch

import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from classes.vehicle import Vehicle

import carla

class TestVehicle(Vehicle):
    def __init__(self, blueprint_lib, carla_world, spawn, destination, transform, blueprint, world, DT, extra, u_nom, alpha, max_acc, max_brake, distance, standard_distance):
        self._Vehicle__car = carla_world.spawn_actor()
        self._Vehicle__world = world
        self._Vehicle__actors = carla_world.get_actors()
        self._Vehicle__rules = []
        self._Vehicle__waypoint_num = 5
        self._Vehicle__DT = DT
        self._Vehicle__extra = extra
        self._Vehicle__u_nom = u_nom
        self._Vehicle__alpha = alpha
        self._Vehicle__max_acc = max_acc
        self._Vehicle__max_brake = max_brake
        self._Vehicle__standard_distance = standard_distance
        self._Vehicle__distance = distance
        self._Vehicle__route = (spawn, destination)

def setUpModule():
    global vehicle

    blueprint_lib = Mock()
    carla_world = Mock()
    car = Mock()

    carla_world.spawn_actor.return_value = car
    carla_world.get_actors.return_value = []

    spawn = Mock()
    destination = Mock()
    transform = Mock()
    blueprint = Mock()
    world = Mock()
    DT = 0
    extra = 0
    u_nom = 0
    alpha = 0
    max_acc = 0
    max_brake = 0
    distance = 0
    standard_distance = 0

    vehicle = TestVehicle(blueprint_lib, carla_world, spawn, destination, transform, blueprint, world, DT, extra, u_nom, alpha, max_acc, max_brake, distance, standard_distance)

class MapeInitTest(unittest.TestCase):
    @patch("classes.vehicle.Executor")
    @patch("classes.vehicle.Planner")
    @patch("classes.vehicle.Analyzer")
    @patch("classes.vehicle.Monitor")
    def test_mape_init(self, mock_monitor, mock_analyzer, mock_planner, mock_executor):
        transform = Mock()
        blueprint = Mock()
        blueprint_lib = Mock()

        mock_monitor.monitor.return_value = {"current_waypoint_num": 0}
        mock_analyzer.analyze.return_value = {"new_waypoint": 1}

        vehicle.mape_init(transform, blueprint, blueprint_lib)

        self.assertTrue(mock_executor.return_value.execute.called)

class MapeDriveTest(unittest.TestCase):
    def test_mape_drive(self):
        mock_monitor = Mock()
        vehicle._Vehicle__monitor_class = mock_monitor

        mock_monitor.monitor.return_value = {"current_waypoint_num": 0}

        mock_analyzer = Mock()
        vehicle._Vehicle__analyzer_class = mock_analyzer

        mock_analyzer.analyze.return_value = {"new_waypoint": 1}

        mock_planner = Mock()
        vehicle._Vehicle__planner_class = mock_planner
        
        mock_planner.plan.return_value = None

        mock_executor = Mock()
        vehicle._Vehicle__executor_class = mock_executor

        vehicle.mape_drive()

        self.assertTrue(mock_executor.execute.called)

if __name__ == "__main__":
    unittest.main()
