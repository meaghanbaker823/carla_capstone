import carla
import traceback
import math

"""
========
AvoidPedestrians Class - WIP

__init__ creates instance and creates list to manage pedestrians

react_to_walkers()                 | moves the vehicle based on the pedestrians

is_light_close()                 | detects if pedestrian is in cars path
========
"""
class AvoidPedestrians():
    def __init__(self, actors):
        self.__pedestrians = []
        self.__response = ""
        self.set_pedestrians(actors)

    def get_pedestrians(self):
        return self.__pedestrians
    def get_response(self):
        return self.__response

    def set_response(self, response):
        self.__response = response
    def set_pedestrians(self, actors):
        self.__pedestrians = actors.filter('walker.pedestrian.*')


    def react_to_walkers(self, car):
        action = ""
        for walker in self.get_pedestrians():
            if (self.is_pedestrian_close(car, walker, 30, 40)):
                action = "stop"
            else:
                action = "drive"
        return action

    def is_pedestrian_close(self, car_check, walker_check, target_distance, target_angle):
        try:
            assert type(car_check) == carla.Vehicle
            assert type(walker_check) == carla.Walker
            assert target_distance >= 0
            assert -180 <= target_angle <= 180
        except:
            print(traceback.format_exc())

        car = car_check.get_transform().get_forward_vector()
        walker = walker_check.get_location() - car_check.get_location()

        dot_product = (car.x * walker.x) + (car.y * walker.y) + (car.z * walker.z)
        magnitude_car = math.sqrt(car.x**2 + car.y**2 + car.z**2)
        magnitude_walker = math.sqrt(walker.x**2 + walker.y**2 + walker.z**2)

        angle_deg = abs(math.degrees(math.acos(max(-1.0, min(1.0, (dot_product / (magnitude_car * magnitude_walker)))))))
        distance = car_check.get_location().distance(walker_check.get_location())

        return ((angle_deg < target_angle) and (distance < target_distance))
