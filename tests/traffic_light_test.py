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
        self.traffic_light.set_color("Red")
        
        self.assertEqual(self.traffic_light.process_color(self.car), "") 
    
# react to color
class ReactToColorTest(unittest.TestCase):
    def setUp(self):
        self.light = Mock()
    def react_to_color(self):
        action = ""
        if(self.light.get_color() == "Green"):
            action = "drive"
        else:
            action = "stop"

        return action
    # test when light is green
    def test_light_green(self):
        self.light.get_color.return_value = "Green"
        self.assertEqual(self.react_to_color(), "drive")

    # test when light is not green
    def test_light_not_green(self):
        self.light.get_color.return_value = "Yellow"
        self.assertEqual(self.react_to_color(), "stop")

# is light close
class IsLightCloseTest(unittest.TestCase):
    def setUp(self):
        self.car_check = Mock()
        self.light_check = Mock()
        self.target_dist = 10
        self.target_angle = 40
        self.light = Mock()
        self.transform_mock = Mock()
        self.car = Mock()


    def is_light_close(self, car_check, light_check, target_distance, target_angle):
        self.car.x = car_check.get_transform().get_forward_vector()[0]
        self.car.y = car_check.get_transform().get_forward_vector()[1]
        self.car.z = car_check.get_transform().get_forward_vector()[2]
        self.light.x = light_check.get_location()[0] - car_check.get_location()[0]
        self.light.y = light_check.get_location()[1] - car_check.get_location()[1]
        self.light.z = light_check.get_location()[2] - car_check.get_location()[2]
    
        dot_product = self.car.x * self.light.x + self.car.y * self.light.y + self.car.z * self.light.z
        magnitude_car = math.sqrt(self.car.x**2 + self.car.y**2 + self.car.z**2)
        magnitude_light = math.sqrt(self.light.x**2 + self.light.y**2 + self.light.z**2)

        angle_deg = abs(math.degrees(math.acos(max(-1.0, min(1.0, (dot_product / (magnitude_car * magnitude_light)))))))
        distance = car_check.distance(light_check.get_location())
        return ((angle_deg < target_angle) and (distance < target_distance))
    
    # angle deg not smaller than target angle, but distance is smaller than target
    def test_case_1(self):
        # setting the angle
        self.car_check.get_transform.return_value = self.transform_mock
        self.transform_mock.get_forward_vector.return_value = (0, 3, 0)
        self.light_check.get_transform.return_value = (10, 0, 0)
        self.car_check.get_location.return_value = (8, 0, 0)

        # setting the distance
        self.light_check.get_location.return_value = (10 , 0, 0)
        self.car_check.distance.return_value = 2
        self.assertEqual(self.is_light_close(self.car_check, self.light_check, self.target_dist, self.target_angle), False)

    # angle deg smaller than target angle and distance < target dist
    def test_case_2(self):
        # setting the angle
        self.car_check.get_transform.return_value = self.transform_mock
        self.transform_mock.get_forward_vector.return_value = (10, 3, 0)
        self.light_check.get_transform.return_value = (10, 0, 0)
        self.car_check.get_location.return_value = (8, 0, 0)

        # setting the distance
        self.light_check.get_location.return_value = (10 , 0, 0)
        self.car_check.distance.return_value = 2
        self.assertEqual(self.is_light_close(self.car_check, self.light_check, self.target_dist, self.target_angle), True)
    
    # angle deg not smaller than target angle and distance >= target dist
    def test_case_3(self):
        # setting the angle
        self.car_check.get_transform.return_value = self.transform_mock
        self.transform_mock.get_forward_vector.return_value = (0, 3, 0)
        self.light_check.get_transform.return_value = (10, 0, 0)
        self.car_check.get_location.return_value = (8, 0, 0)

        # setting the distance
        self.light_check.get_location.return_value = (10 , 0, 0)
        self.car_check.distance.return_value = 13
        self.assertEqual(self.is_light_close(self.car_check, self.light_check, self.target_dist, self.target_angle), False)
   
    # angle deg < target angle and distance >= target dist
    def test_case_4(self):
        # setting the angle
        self.car_check.get_transform.return_value = self.transform_mock
        self.transform_mock.get_forward_vector.return_value = (10, 3, 0)
        self.light_check.get_transform.return_value = (10, 0, 0)
        self.car_check.get_location.return_value = (8, 0, 0)

        # setting the distance
        self.light_check.get_location.return_value = (10 , 0, 0)
        self.car_check.distance.return_value = 13
        self.assertEqual(self.is_light_close(self.car_check, self.light_check, self.target_dist, self.target_angle), False)

if __name__ == "__main__":
    unittest.main()




