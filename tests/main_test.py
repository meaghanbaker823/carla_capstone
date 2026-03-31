import unittest
from unittest.mock import Mock
from unittest.mock import patch
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from main import main
from classes.collision_exception import CollisionErr
from classes.route_done import RouteDone
from classes.world import World
import carla

class MainTest(unittest.TestCase):
    def setUp(self):
       self.client = Mock(spec = carla.libcarla.Client)
       self.spts = [Mock(spec = carla.libcarla.Transform), Mock(spec = carla.libcarla.Transform)]
       self.spawn = Mock(spec = carla.libcarla.Transform)
       self.blueprints = [Mock(speck = carla.libcarla.ActorBlueprint), Mock(speck = carla.libcarla.ActorBlueprint)]

       self.spectator = Mock()
       self.vehicle = Mock()
       self.car = Mock(spec = carla.libcarla.Vehicle)

    # assert init world called
    @patch("main.World")
    def test_init_world(self, MockWorld):
        MockWorld.side_effect = KeyboardInterrupt
        try:
            main()
        except KeyboardInterrupt:
            pass
        MockWorld.assert_called_once()
    
    # spectator success
    @patch("main.World")
    def test_spectator_success(self, MockWorld):
        # init world mocking
        instance = MockWorld.return_value
        instance.init_world.return_value = (self.client, self.spts, self.spawn, self.blueprints)

        # id spec is successful, then it should reach init_actors
        instance.init_spectator.return_value = self.spectator

        instance.init_actors.side_effect = KeyboardInterrupt

        try: 
            with patch("builtins.print") as mock_print:
                main()
        except KeyboardInterrupt:
            pass

        instance.init_actors.assert_called_once()

    # spectator failure
    @patch("main.World")
    def test_spectator_failure(self, MockWorld):   
        instance = MockWorld.return_value
        # init world mocking
        instance.init_world.return_value = (self.client, self.spts, self.spawn, self.blueprints)

        # if spec fails, then it should not reach init_actors
        instance.init_spectator.side_effect = TypeError
      
        try:
            with patch("builtins.print") as mock_print:
                main()
        except TypeError:
            pass
        # will still print type error bc of traceback in main (showing general exception case)
        instance.init_actors.assert_not_called()
        
    # init actors success
    @patch("main.World")
    def test_init_actors_success(self, MockWorld):
        instance = MockWorld.return_value
        # init world mocking
        instance.init_world.return_value = (self.client, self.spts, self.spawn, self.blueprints)

        # init spectator
        instance.init_spectator.return_value = self.spectator

        # setting init actors
        instance.init_actors.return_value = (self.vehicle, self.car)
        instance.get_world.side_effect = KeyboardInterrupt
        
        try: 
            with patch("builtins.print") as mock_print:
                main()
        except KeyboardInterrupt:
            pass

        # will still print type error bc of traceback in main (showing general exception case)
        instance.main_loop.assert_called_once()

    # init actors failure
    # test collision err
    # test routedone Err
    # test keyboard interupt
    # assert that clear world called

        



if __name__ == "__main__":
    unittest.main()