import unittest
from unittest.mock import Mock
from unittest.mock import patch
import math
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from classes.trafficLight import TrafficLight
import carla

class ProcessColorTest(unittest.TestCase):
    def setUp(self):
        self.car = Mock(spec = carla.libcarla.Vehicle)
        self.lights = [Mock(),]
        self.lights[0].length = 3

        self.new_color = 'Red'
        self.new_response = "stop"
        with patch("carla.ActorList") as mock_actor_list:
            mock_actor_list.filter.return_value = self.lights
            self.traffic_light = TrafficLight(mock_actor_list)
        
    @patch.object(TrafficLight, "is_light_close")
    def test_light0(self, mock_is_light_close):
        # test going from red to red
        mock_is_light_close.return_value = True
        mock_state = Mock()
        mock_state.name = "Red"
        self.lights[0].get_state.return_value = mock_state
        self.traffic_light.set_color("Red")
        
        self.assertEqual(self.traffic_light.process_color(self.car), "")        


    @patch.object(TrafficLight, "is_light_close")
    def test_light1(self, mock_is_light_close):
        # test going from Green to red
        mock_is_light_close.return_value = True
        # sets new color
        mock_state = Mock()
        mock_state.name = "Red"
        self.lights[0].get_state.return_value = mock_state
        # sets old color
        self.traffic_light.set_color("Green")
        
        self.assertEqual(self.traffic_light.process_color(self.car), "stop") 

    @patch.object(TrafficLight, "is_light_close")
    def test_light2(self, mock_is_light_close):
        # test going from yellow to red
        mock_is_light_close.return_value = False
        # sets new color
        mock_state = Mock()
        mock_state.name = "Red"
        self.lights[0].get_state.return_value = mock_state
        # sets old color
        self.traffic_light.set_color("Yellow")
        
        self.assertEqual(self.traffic_light.process_color(self.car), "") 
    
# react to color
class ReactToColorTest(unittest.TestCase):
    def setUp(self):
        self.lights = [Mock(),]
        with patch("carla.ActorList") as mock_actor_list:
            mock_actor_list.filter.return_value = self.lights
            self.traffic_light = TrafficLight(mock_actor_list)

    # test when light is green
    def test_light_green(self):
        self.traffic_light.set_color("Green")
        self.assertEqual(self.traffic_light.react_to_color(), "drive")

    # test when light is not green
    def test_light_not_green(self):
        self.traffic_light.set_color("Yellow")
        self.assertEqual(self.traffic_light.react_to_color(), "stop")

# is light close
class IsLightCloseTest(unittest.TestCase):
    def setUp(self):
        self.lights = [Mock(),]
        with patch("carla.ActorList") as mock_actor_list:
            mock_actor_list.filter.return_value = self.lights
            self.traffic_light = TrafficLight(mock_actor_list)

        self.car_check = Mock(spec = carla.libcarla.Vehicle)
        self.light_check = Mock(spec = carla.TrafficLight)
        self.target_dist = 10
        self.target_angle = 40

        self.car_location = carla.Location(x = 8, y = 0, z = 0)
        self.light_location = carla.Location(x = 10, y = 0, z = 0)

        self.car_transform = carla.Transform(self.car_location,
                                              carla.Rotation(pitch = 0, yaw = 0, roll = 0))
        self.light_transform = carla.Transform(self.light_location,
                                              carla.Rotation(pitch = 0, yaw = 0, roll = 0))

        self.car_check.get_transform.return_value = self.car_transform
        self.light_check.get_transform.return_value = self.light_transform
        self.car_check.get_location.return_value = self.car_location
        self.light_check.get_location.return_value = self.light_location

    
    # angle deg not smaller than target angle, but distance is smaller than target
    def test_case_1(self):
        self.car_location = carla.Location(x = 8, y = 0, z = 0)
        self.light_location = carla.Location(x = 10, y = 0, z = 0)

        self.car_transform = carla.Transform(self.car_location,
                                              carla.Rotation(pitch = 0, yaw = 90, roll = 0))
        self.light_transform = carla.Transform(self.light_location,
                                              carla.Rotation(pitch = 0, yaw = 0, roll = 0))

        self.car_check.get_transform.return_value = self.car_transform
        self.light_check.get_transform.return_value = self.light_transform
        self.car_check.get_location.return_value = self.car_location
        self.light_check.get_location.return_value = self.light_location
        # setting the angle

        self.assertEqual(self.traffic_light.is_light_close(self.car_check, self.light_check, self.target_dist, self.target_angle), False)

    # angle deg smaller than target angle and distance < target dist
    def test_case_2(self):
        # setting the angle with yaw
        # setting distance with location

        self.car_location = carla.Location(x = 8, y = 0, z = 0)
        self.light_location = carla.Location(x = 10, y = 0, z = 0)

        self.car_transform = carla.Transform(self.car_location,
                                              carla.Rotation(pitch = 0, yaw = 0, roll = 0))
        self.light_transform = carla.Transform(self.light_location,
                                              carla.Rotation(pitch = 0, yaw = 0, roll = 0))

        self.car_check.get_transform.return_value = self.car_transform
        self.light_check.get_transform.return_value = self.light_transform
        self.car_check.get_location.return_value = self.car_location
        self.light_check.get_location.return_value = self.light_location

        self.assertEqual(self.traffic_light.is_light_close(self.car_check, self.light_check, self.target_dist, self.target_angle), True)
    
    # angle deg not smaller than target angle and distance >= target dist
    def test_case_3(self):
        # setting the angle with yaw and distance with location

        self.car_location = carla.Location(x = 0, y = 0, z = 0)
        self.light_location = carla.Location(x = 13, y = 0, z = 0)

        self.car_transform = carla.Transform(self.car_location,
                                              carla.Rotation(pitch = 0, yaw = 90, roll = 0))
        self.light_transform = carla.Transform(self.light_location,
                                              carla.Rotation(pitch = 0, yaw = 0, roll = 0))

        self.car_check.get_transform.return_value = self.car_transform
        self.light_check.get_transform.return_value = self.light_transform
        self.car_check.get_location.return_value = self.car_location
        self.light_check.get_location.return_value = self.light_location
        # setting the angle

        self.assertEqual(self.traffic_light.is_light_close(self.car_check, self.light_check, self.target_dist, self.target_angle), False)
   
    # angle deg < target angle and distance >= target dist
    def test_case_4(self):
        self.car_location = carla.Location(x = 0, y = 0, z = 0)
        self.light_location = carla.Location(x = 13, y = 0, z = 0)

        self.car_transform = carla.Transform(self.car_location,
                                              carla.Rotation(pitch = 0, yaw = 0, roll = 0))
        self.light_transform = carla.Transform(self.light_location,
                                              carla.Rotation(pitch = 0, yaw = 0, roll = 0))

        self.car_check.get_transform.return_value = self.car_transform
        self.light_check.get_transform.return_value = self.light_transform
        self.car_check.get_location.return_value = self.car_location
        self.light_check.get_location.return_value = self.light_location
        # setting the angle

        self.assertEqual(self.traffic_light.is_light_close(self.car_check, self.light_check, self.target_dist, self.target_angle), False)

if __name__ == "__main__":
    unittest.main()




