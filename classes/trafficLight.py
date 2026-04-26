import carla
import math

class TrafficLight():
    """
    A class to collect all traffic lights that are within the range of the car and give its correct reaction
    """
    def __init__(self, actors):
        """
        Initializes the variables needed for the TrafficLight class
        \n\tINPUT(S): actors: the actors list for carla,
        \n\tOUTPUT(S): N/A
        """        
        self.__lights = []
        self.__color = "unknown"
        self.__response = ""
        self.set_lights(actors)

    def get_color(self):
        """
        Getter for self.__color
        \n\tINPUT(S): N/A
        \n\tOUTPUT(S): returns the color of the traffic light
        """   
        return self.__color
    
    def get_lights(self):
        """
        Getter for self.__lights
        \n\tINPUT(S): N/A
        \n\tOUTPUT(S): returns a list of the traffic lights on the carla map
        """   
        return self.__lights
    
    def get_response(self):
        """
        Getter for self.__response
        \n\tINPUT(S): N/A
        \n\tOUTPUT(S): returns how to respond to the traffic light
        """   
        return self.__response
    
    def set_lights(self, actors):
        """
        Setter for self.__lights
        \n\tINPUT(S): actors: the carla actor list
        \n\tOUTPUT(S): the list containing all of the traffic lights on the carla map
        """   
        self.__lights = actors.filter('traffic.traffic_light*')

    def set_response(self, response):
        """
        Setter for self.__response
        \n\tINPUT(S): response: a string containing how to repond to a traffic light
        \n\tOUTPUT(S): N/A
        """   
        self.__response = response

    def set_color(self, color):
        """
        Setter for self.__color
        \n\tINPUT(S): color: the color of the traffic light (string)
        \n\tOUTPUT(S): N/A
        """   
        self.__color = color

    def process_color(self, car):
        """
        Processes the color of each of the traffic lights
        \n\tINPUT(S): car: the carla vehicle
        \n\tOUTPUT(S): the response of the vehicle to the light
        """
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
        """
        Decides what action should be taken based on the traffic color
        \n\tINPUT(S): N/A
        \n\tOUTPUT(S): the action the car should take in response to the light (string)
        """   
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
        """
        Determines if the traffic light is close and in the right angle for the car
        \n\tINPUT(S): car_check: the carla vehicle,
                      light_check: a carla traffic light,
                      target_distance: the distance in which the car will respond to a light,
                      target_angle: the angle in which the car will respond to a light
        \n\tOUTPUT(S): a boolean - True if the light is in distance and angle, False otherwise
        """   
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