import unittest
from unittest.mock import Mock
from unittest.mock import MagicMock
from unittest.mock import patch

import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from classes.collisionSensor import CollisionSensor

import carla

def setUpModule():
    global collision_sensor

    relative_transform = Mock(spec=carla.libcarla.Transform)

    parent_actor = Mock()
    parent_actor.__str__ = Mock(return_value="car")
    
    blueprint = Mock()
    blueprint.__str__ = Mock(return_value="sensor.other.collision")

    world = Mock()
    world.get_actors.return_value = ["car"]

    blueprint_lib = Mock()
    blueprint_lib.find.return_value = "sensor.other.collision"

    collision_sensor = CollisionSensor(relative_transform, parent_actor, blueprint, world, blueprint_lib)

    global collisions

    collisions = Mock()
    collision_sensor._CollisionSensor__collisions = collisions

class CollisionDetectTest(unittest.TestCase):
    def test_collision_detect_pass(self):
        event = Mock(spec=carla.libcarla.CollisionEvent)

        collisions.__len__ = Mock(side_effect = [5, 6])

        collision_sensor.collision_detect(event)

        self.assertTrue(collisions.append.called)

    def test_collision_detect_fail(self):
        event = Mock(spec=carla.libcarla.CollisionEvent)

        collisions.__len__ = Mock(side_effect = [5, 5])

        with self.assertRaises(AssertionError):
            collision_sensor.collision_detect(event)

class ListenTest(unittest.TestCase):
    @patch.object(CollisionSensor, "get_sensor")
    def test_listen(self, mock_get_sensor):
       collision_sensor.listen()

       self.assertTrue(mock_get_sensor.called)


if __name__ == "__main__":
    unittest.main()
