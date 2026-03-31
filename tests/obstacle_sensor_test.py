import unittest
from unittest.mock import Mock
from unittest.mock import MagicMock
from unittest.mock import patch

import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from classes.obstacleSensor import ObstacleSensor

import carla

def setUpModule():
    global obstacle_sensor

    relative_transform = Mock(spec=carla.libcarla.Transform)

    parent_actor = Mock()
    parent_actor.__str__ = Mock(return_value="car")
    
    blueprint = Mock()
    blueprint.__str__ = Mock(return_value="sensor.other.obstacle")

    world = Mock()
    world.get_actors.return_value = ["car"]

    blueprint_lib = Mock()
    blueprint_lib.find.return_value = "sensor.other.obstacle"

    obstacle_sensor = ObstacleSensor(relative_transform, parent_actor, blueprint, world, blueprint_lib)

    global other_actors
    global detections

    other_actors = MagicMock()
    detections = MagicMock()
    obstacle_sensor._ObstacleSensor__other_actors = other_actors
    obstacle_sensor._ObstacleSensor__detections = detections

class DeleteOldDetectionsTest(unittest.TestCase):
    def test_delete_old_detections_pass(self):
        other_actors.__len__ = Mock(side_effect = [5, 4])
        detections.__len__ = Mock(side_effect = [5, 4])

        obstacle_sensor.delete_old_detection()
       
        self.assertTrue(other_actors.pop.called)
        self.assertTrue(detections.pop.called)

    def test_delete_old_detections_fail(self):
        other_actors.__len__ = Mock(side_effect = [5, 5])
        detections.__len__ = Mock(side_effect = [5, 5])

        with self.assertRaises(AssertionError):
            obstacle_sensor.delete_old_detection()

class ObstacleDetectTest(unittest.TestCase):
    def test_obstacle_detect_pass(self):
        event = Mock(spec=carla.libcarla.ObstacleDetectionEvent)

        detections.__len__ = Mock(side_effect = [5, 6])

        event.other_actor = "obstacle"

        other_actors.__iter__.return_value = []

        event.distance = 5

        obstacle_sensor.obstacle_detect(event)

        self.assertTrue(detections.append.called)
        self.assertTrue(other_actors.append.called)

    def test_obstacle_detect_fail(self):
        event = Mock(spec=carla.libcarla.ObstacleDetectionEvent)

        detections.__len__ = Mock(side_effect = [5, 5])

        event.other_actor = "obstacle"

        other_actors.__iter__.return_value = []

        event.distance = 5

        with self.assertRaises(AssertionError):
            obstacle_sensor.obstacle_detect(event)

class ListenTest(unittest.TestCase):
    @patch.object(ObstacleSensor, "get_sensor")
    def test_listen(self, mock_get_sensor):
       obstacle_sensor.listen()

       self.assertTrue(mock_get_sensor.called)

if __name__ == "__main__":
    unittest.main()
