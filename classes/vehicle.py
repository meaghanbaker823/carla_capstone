import random
import traceback
import carla
import math

from classes.obstacleSensor import ObstacleSensor
from classes.collisionSensor import CollisionSensor
from classes.trafficLight import TrafficLight
from classes.pedestrianAvoidance import AvoidPedestrians
from classes.globalRoutePlanner import GlobalRoutePlanner
from classes.monitor import Monitor
from classes.analyzer import Analzyer
from classes.planner import Planner
from classes.executor import Executor

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
    def __init__(self, blueprint_lib, carla_world, spawn, destination, transform, blueprint, world):
        self.__car = carla_world.spawn_actor(random.choice(blueprint_lib.filter('vehicle.bmw.*')), spawn)
        self.__actors = carla_world.get_actors()
        self.__world = world
        self.__carla_world = carla_world
        self.__sensors = []
        self.__rules = []
        self.__checks = []
        self.__waypoint_num = None
        self.__global_route_planner = GlobalRoutePlanner(self.__carla_world.get_map(), 1)
        self.__route = self.__global_route_planner.trace_route(spawn.location, destination.location)
        self.mape_init(transform, blueprint, blueprint_lib, 5, 30)
        self.draw_route()

    def draw_route(self):
        for waypoint in self.__route:
            self.__carla_world.debug.draw_string(waypoint[0].transform.location, '^', draw_shadow = False, color = carla.Color(r = 0, g = 0, b = 255), life_time = 90.0, persistent_lines = True)

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

        new = 0

        for check in self.get_checks():
            if(check.speed_check(current, self.get_speed_limit()) != -1):
                new = check.speed_check(current, self.get_speed_limit())
                break
        
        try:
            assert 0 <= new <= 1
        except:
            print(traceback.format_exc())
        return new
        
    def mape_init(self, transform, blueprint, blueprint_lib, initial_waypoint_num, speed_limit):
        try:
            self.__monitor_class = Monitor(transform, self.__car, blueprint, self.__world, blueprint_lib, self.__actors, self.__route)
            self.__analyzer_class = Analzyer()
            self.__planner_class = Planner()
            self.__executor_class = Executor()
            monitor_info = self.__monitor_class.monitor(initial_waypoint_num)
            analyzer_info = self.__analyzer_class.analyze(self.get_car(), self.get_rules(), monitor_info)
            self.__waypoint_num = analyzer_info["new_waypoint"]
            plan_info = self.__planner_class.plan(analyzer_info, speed_limit, self.get_checks())
            self.__executor_class.execute(plan_info, self.get_car())
            return True

        except:
            print(traceback.format_exc())
            return False



    def mape_drive(self, speed_limit):     
        try:
            monitor_info = self.__monitor_class.monitor(self.__waypoint_num)
            analyzer_info = self.__analyzer_class.analyze(self.get_car(), self.get_rules(), monitor_info)
            self.__waypoint_num = analyzer_info["new_waypoint"]
            plan_info = self.__planner_class.plan(analyzer_info, speed_limit, self.get_checks())
            self.__executor_class.execute(plan_info, self.get_car())
            print(self.__executor_class.notify())
    
            return True

        except:
            print(traceback.format_exc())
            return False


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
            self.get_car().apply_control(carla.VehicleControl(throttle = estimated_throttle, steer=steering_angle, brake= brake_force))
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

