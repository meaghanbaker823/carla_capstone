import unittest
from unittest.mock import Mock
from unittest.mock import patch
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from main import main
from classes.collisionException import CollisionErr
from classes.routeDone import RouteDone
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
        instance.init_actors.assert_not_called()
        
    # init actors success and keyboard interupt
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
                mock_print.assert_called_once_with(" Keyboard Interrupt")
        except KeyboardInterrupt:
            pass

        instance.main_loop.assert_called_once()

    # init actors failure and traceback info
    @patch("main.World")
    def test_init_actors_failure(self, MockWorld):
        instance = MockWorld.return_value
        # init world mocking
        instance.init_world.return_value = (self.client, self.spts, self.spawn, self.blueprints)

        # init spectator
        instance.init_spectator.return_value = self.spectator

        # if init actors fails, then should never run main loop
        instance.init_actors.side_effect = TypeError
        
        try: 
            with patch("builtins.print") as mock_print:
                main()
                mock_print.assert_called_once_with(('Traceback (most recent call last):\n  File "/home/cae-user/group3AV/carla_capstone/main.py", line 25, in main\n    vehicle, car = world.init_actors(spawn, blueprint_lib, spts, world, scanning_distance, dt, standard_distance, u_nom, alpha, max_acc, max_brake, distance, extra)\n  File "/usr/lib/python3.10/unittest/mock.py", line 1114, in __call__\n    return self._mock_call(*args, **kwargs)\n  File "/usr/lib/python3.10/unittest/mock.py", line 1118, in _mock_call\n    return self._execute_mock_call(*args, **kwargs)\n  File "/usr/lib/python3.10/unittest/mock.py", line 1173, in _execute_mock_call\n    raise effect\nTypeError\n'))
        except TypeError:
            pass

        instance.main_loop.assert_not_called()

    # test collision err
    @patch("main.World")
    def test_collision_err(self, MockWorld):
        instance = MockWorld.return_value
        # init world mocking
        instance.init_world.return_value = (self.client, self.spts, self.spawn, self.blueprints)

        # init spectator
        instance.init_spectator.return_value = self.spectator

        # setting init actors
        instance.init_actors.return_value = (self.vehicle, self.car)
        instance.main_loop.side_effect = CollisionErr("")
        
        try: 
            with patch("builtins.print") as mock_print:
                main()
                mock_print.assert_called_once_with("Collision")
        except CollisionErr:
            pass

        instance.get_world.assert_not_called()

    # test routedone Err
    @patch("main.World")
    def test_route_done(self, MockWorld):
        instance = MockWorld.return_value
        # init world mocking
        instance.init_world.return_value = (self.client, self.spts, self.spawn, self.blueprints)

        # init spectator
        instance.init_spectator.return_value = self.spectator

        # setting init actors
        instance.init_actors.return_value = (self.vehicle, self.car)
        instance.main_loop.side_effect = RouteDone("")
        
        try: 
            with patch("builtins.print") as mock_print:
                main()
                mock_print.assert_called_once_with("Route done!")
        except RouteDone:
            pass

        instance.get_world.assert_not_called()

    # assert that clear world called
    @patch("main.World")
    def test_world_cleared(self, MockWorld):
        instance = MockWorld.return_value
        # init world mocking
        instance.init_world.return_value = (self.client, self.spts, self.spawn, self.blueprints)

        # init spectator
        instance.init_spectator.return_value = self.spectator

        # setting init actors
        instance.init_actors.return_value = (self.vehicle, self.car)
        instance.main_loop.side_effect = RouteDone("")
        
        try: 
            with patch("builtins.print") as mock_print:
                main()
        except RouteDone:
            pass

        instance.clear_world.assert_called_once()
        



if __name__ == "__main__":
    unittest.main()