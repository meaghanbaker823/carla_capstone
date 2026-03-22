from unittest.mock import Mock
from unittest.mock import patch
import unittest

import sys

sys.path.append("..")

from classes.world import World

import carla

def setUpModule():
    global world
    world = World()

class InitSpectatorTest(unittest.TestCase):
    def test_init_spectator(self):
        mock_spawn = Mock(spec=carla.libcarla.Transform)

        mock_spawn.location.x = 0
        mock_spawn.location.y = 0
        mock_spawn.location.z = 0

        return_value = world.init_spectator(mock_spawn)

        location = return_value.get_location()

        self.assertEqual(location.x, 0)
        self.assertEqual(location.y, 0)
        self.assertEqual(location.z, 60)

class SpawnVehicleTest(unittest.TestCase):
    @patch('carla.World.spawn_actor')
    def test_spawn_vehicle(self, mock_carla_world):
        world.spawn_vehicles()

        self.assertTrue(mock_carla_world.called)

class SpawnPedestriansTest(unittest.TestCase):
   @patch('carla.World.spawn_actor')
   def test_spawn_actors(self, mock_carla_world):
       world.spawn_pedestrians()

       self.assertTrue(mock_carla_world.called)

if __name__ == "__main__":
    unittest.main()
