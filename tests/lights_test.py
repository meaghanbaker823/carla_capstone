import unittest
from unittest.mock import Mock

class LightTest(unittest.TestCase):
    def test_light(self):
        car = Mock()
        lights = Mock()

        lights.length = 3
        lights.get_lights = [Mock(), Mock(), Mock()]

        # setting all of the api call values for the mock object
        lights.get_lights[0].is_light_close.return_value = True
        lights.get_lights[0].get_color.return_value = "Red"
        lights.get_lights[0].get_response.return_value = ""

        lights.get_lights[1].is_light_close.return_value = True
        lights.get_lights[1].get_color.return_value = "Green"
        lights.get_lights[1].get_response.return_value = ""

        lights.get_lights[2].is_light_close.return_value = False
        lights.get_lights[2].get_color.return_value = "Yellow"
        lights.get_lights[2].get_response.return_value = ""
        

        new_color = 'Red'
        new_response = "stop"

        for light in lights.get_lights:
            if light.is_light_close():
                old_color = light.get_color()
                light.get_color.return_value = new_color

                if old_color != light.get_color():
                    light.get_response.return_value = new_response
                    break

            else:
                light.get_response.return_value = ""    

        self.assertEqual(lights.get_lights[0].get_response(), "")  
        self.assertEqual(lights.get_lights[1].get_response(), 'stop') 
        self.assertEqual(lights.get_lights[2].get_response(), "")        
                        


if __name__ == "__main__":
    unittest.main()




