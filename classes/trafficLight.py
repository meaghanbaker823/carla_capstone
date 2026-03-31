import carla
import math

"""
===========
TrafficLight Class

__init__ creates instance and creates list to manage traffic lights

process_color()                  | sets color by processing the state of the light

react_to_color()                 | moves the vehicle based on the color

is_light_close()                 | detects if light is applicable

===========
"""
class TrafficLight():
    def __init__(self, actors):
        self.__lights = []
        self.__color = "unknown"
        self.__response = ""
        self.set_lights(actors)

    def get_color(self):
        return self.__color
    def get_lights(self):
        return self.__lights
    def get_response(self):
        return self.__response
    
    def set_lights(self, actors):
        self.__lights = actors.filter('traffic.traffic_light*')
    def set_response(self, response):
        self.__response = response
    def set_color(self, color):
        self.__color = color

    def process_color(self, car):
        try:
            assert isinstance(car, carla.libcarla.Vehicle)
        except:
            raise

        # this portion for the testing assignment
        for light in self.get_lights():
            if(self.is_light_close(car, light, 10, 40)):
                old_color = self.get_color()
                self.set_color(light.get_state().name)

                if(old_color != self.get_color()):
                    self.set_response(self.react_to_color())
                    break
            else:
                self.set_response("")
  
        return self.get_response()

    def react_to_color(self):
        action = ""
        if(self.get_color() == "Green"):
            action = "drive"
        else:
            action = "stop"
        
        try:
            assert type(action) == str
        except:
            raise

        return action
    
    def is_light_close(self, car_check, light_check, target_distance, target_angle):
        try:
            assert isinstance(car_check,  carla.Vehicle)
            assert isinstance(light_check, carla.TrafficLight)
            assert target_distance >= 0
            assert -180 <= target_angle <= 180
        except:
            raise
        
        car = car_check.get_transform().get_forward_vector()
        light = light_check.get_location() - car_check.get_location()
    
        dot_product = car.x * light.x + car.y * light.y + car.z * light.z
        magnitude_car = math.sqrt(car.x**2 + car.y**2 + car.z**2)
        magnitude_light = math.sqrt(light.x**2 + light.y**2 + light.z**2)

        angle_deg = abs(math.degrees(math.acos(max(-1.0, min(1.0, (dot_product / (magnitude_car * magnitude_light)))))))
        distance = car_check.get_location().distance(light_check.get_location())

        return ((angle_deg < target_angle) and (distance < target_distance))