import random
import traceback
import carla
import math

from classes.obstacleSensor import ObstacleSensor
from classes.collisionSensor import CollisionSensor
from classes.trafficLight import TrafficLight

"""
===========
Vehicle Class

__init__ creates instance and creates list to manage a vehicle

control_loop()                  | sets color by processing the state of the light

set_sensors()                   | moves the vehicle based on the color

is_car_moving()                 | detects if light is applicable

maintain_speed()                | adjusts throttle based on current speed to adjust to current speed

drive()                         | drives the car based on navigator

decelerate()                    | slows down car with brake

stop_car()                      | stops car by applying full brake

fix_lane()                      | adjusts car into the center of the lane

avoid_obstacles()               | will avoid obstacles detected
===========
"""
control_flag = True
class Vehicle: 
    def __init__(self, blueprint_lib, world_map, spawn, navigator):
        self.__car = world_map.spawn_actor(random.choice(blueprint_lib.filter('vehicle.bmw.*')), spawn)
        self.__actors = world_map.get_actors()
        self.__sensors = []
        self.__navigator = navigator
        self.__speed_limit = 30
        self.__rules = []
        self.__checks = []

    def get_sensors(self):
        return self.__sensors
    
    def add_sensor(self, sensor):
        self.__sensors.append(sensor)
    
    def get_rules(self):
        return self.__rules
    
    def set_rules(self, rules):
        self.__rules = rules
    
    def get_checks(self):
        return self.__checks
    
    def set_checks(self, checks):
        self.__checks = checks

    def get_navigator(self):
        return self.__navigator
  
    def get_car(self):
        return self.__car
    
    def get_sensors(self):
        return self.__sensors
    
    def get_speed_limit(self):
        return self.__speed_limit
    
    def get_actors(self):
        return self.__actors
    
    def set_speed_limit(self ,speed):
        self.__speed_limit = speed

    def set_sensors(self, transform, actor, blueprint, world, blueprint_lib):
        self.add_sensor(ObstacleSensor(transform[0], actor, blueprint[0], world, blueprint_lib))
        self.add_sensor(CollisionSensor(transform[1], actor, blueprint[1], world, blueprint_lib))
    
        for sensor in self.__sensors:
            sensor.listen()

    # refactor control loop
    def control_loop(self):
        global control_flag
        car_changed = False
        traffic_lights = TrafficLight(self.get_actors())
        for rule in self.__rules:
            # if false, then rule not passed
            if not rule.rule_flag(traffic_lights):
                if not control_flag:
                    return False
                car_changed = True

        if(not car_changed):
            if(not self.drive()):
                return False
    
    def is_car_moving(self):
        return self.get_car().get_velocity()
 

    def maintain_speed(self, current):
        try:
            assert current >= 0
        except:
            print(traceback.format_exc())

        for check in self.get_checks():
            if(check.speed_check(current, self.get_speed_limit()) != -1):
                new = check.speed_check(current, self.get_speed_limit())
                break
        
        try:
            assert 0 <= new <= 1
        except:
            print(traceback.format_exc())
        return new
        
    def drive(self):
        try:
            if self.__navigator.get_cur_waypoint_num() >= (len(self.__navigator.get_route()) - 6):
                return False
            self.__navigator.advance_waypoint(self.__car)
            v = self.get_car().get_velocity()                                   # velocity is a 3d vector in m/s
            speed = round(3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2),0)          # speed in kilometers/hr 
            
            waypt_nm, steering_angle = self.__navigator.get_proper_angle(self.__car, self.__navigator.get_cur_waypoint_num(), self.__navigator.get_route()) 
            
            self.__navigator.set_cur_waypoint(waypt_nm)
            steering_angle /= 75
            
            estimated_throttle = self.maintain_speed(speed)
            if (estimated_throttle == 0):
                brake_force = 1.0
            else:
                brake_force = 0
            self.get_car().apply_control(carla.VehicleControl(throttle=estimated_throttle,steer=steering_angle, brake=brake_force))
            return True
        
        except:
            print(traceback.format_exc())
            return False

    def decelerate(self):
        self.get_car().apply_control(carla.VehicleControl(throttle=0, brake=0.7))

    def avoid_obstacles(self):
        if(self.__sensors[0].get_other_actors()[0].type_id[:-2] == "traffic.speed_limit."):
            self.set_speed_limit(30)
        self.__sensors[0].delete_old_detection()
 
    def stop_car(self):
        if(self.is_car_moving):
            self.get_car().apply_control(carla.VehicleControl(throttle=0,brake=1.0))
