import unittest
from unittest.mock import Mock

class LightTest(unittest.TestCase):

    def lights_to_test(self):
        for light in self.lights.get_lights:
            if light.is_light_close():
                old_color = light.get_color()
                light.get_color.return_value = self.new_color

                if old_color != light.get_color():
                    light.get_response.return_value = self.new_response
                    break

            else:
                light.get_response.return_value = "" 

    def setUp(self):
        self.car = Mock()
        self.lights = Mock()
        self.lights.length = 3
        self.lights.get_lights = [Mock()]

        self.new_color = 'Red'
        self.new_response = "stop"


    def test_light0(self):
        # setting all of the api call values for the mock object
        self.lights.get_lights[0].is_light_close.return_value = True
        self.lights.get_lights[0].get_color.return_value = "Red"
        self.lights.get_lights[0].get_response.return_value = ""

        self.lights_to_test() 

        self.assertEqual(self.lights.get_lights[0].get_response(), "")        

    def test_light1(self):
        self.lights.get_lights[0].is_light_close.return_value = True
        self.lights.get_lights[0].get_color.return_value = "Green"
        self.lights.get_lights[0].get_response.return_value = ""

        self.lights_to_test()

        self.assertEqual(self.lights.get_lights[0].get_response(), 'stop')

    def test_light2(self):
        self.lights.get_lights[0].is_light_close.return_value = False
        self.lights.get_lights[0].get_color.return_value = "Yellow"
        self.lights.get_lights[0].get_response.return_value = ""

        self.lights_to_test()
        self.assertEqual(self.lights.get_lights[0].get_response(), "")
    


                        


if __name__ == "__main__":
    unittest.main()




