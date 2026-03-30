import unittest
from unittest.mock import Mock
from unittest.mock import patch
import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from main import main
from classes.collision_exception import CollisionErr
from classes.route_done import RouteDone

class MainTest(unittest.TestCase):
    def setUp(self):
       pass

    # assert init world called
    @patch("main.World")
    def test_init_world(self, MockWorld):
        MockWorld.side_effect = KeyboardInterrupt
        try:
            main()
        except KeyboardInterrupt:
            pass
        MockWorld.assert_called_once()
    # assert value of control flag
    
    # spectator failure
    # spectator success
    # init actors success
    # init actors failure
    # test collision err
    # test routedone Err
    # test general exception
    # test keyboard interupt
    # assert that clear world called
    def test_case1(self):
        pass
        



if __name__ == "__main__":
    unittest.main()