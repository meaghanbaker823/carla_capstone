from classes.globalRoutePlanner import GlobalRoutePlanner
from classes.monitor import Monitor
from classes.analyzer import Analzyer
from classes.planner import Planner
from classes.executor import Executor

import random
import carla

"""
===========
Vehicle Class

__init__ creates instance and creates list to manage a vehicle

drive()                         | drives the car based on MAPE

===========
"""
control_flag = True
class Vehicle: 
    def __init__(self, blueprint_lib, carla_world, spawn, destination, transform, blueprint, world):
        self.__car = carla_world.spawn_actor(random.choice(blueprint_lib.filter('vehicle.bmw.*')), spawn)
        self.__actors = carla_world.get_actors()
        self.__world = world
        self.__carla_world = carla_world
        self.__rules = []
        self.__checks = []
        self.__waypoint_num = None
        self.__global_route_planner = GlobalRoutePlanner(self.__carla_world.get_map(), 1)
        self.__route = self.__global_route_planner.trace_route(spawn.location, destination.location)
        self.mape_init(transform, blueprint, blueprint_lib, 5, 30)
        self.draw_route()

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
        except:
            raise

    def draw_route(self):
        for waypoint in self.__route:
            self.__carla_world.debug.draw_string(waypoint[0].transform.location, '^', draw_shadow = False, color = carla.Color(r = 0, g = 0, b = 255), life_time = 90.0, persistent_lines = True)
    
    def get_rules(self):
        return self.__rules
    
    def set_rules(self, rules):
        self.__rules = rules

    def get_sensors(self):
        return self.__monitor_class.get_sensors()
    
    def get_checks(self):
        return self.__checks
    
    def set_checks(self, checks):
        self.__checks = checks
  
    def get_car(self):
        return self.__car
    
    def get_actors(self):
        return self.__actors

    def mape_drive(self, speed_limit):     
        try:
            monitor_info = self.__monitor_class.monitor(self.__waypoint_num)
            analyzer_info = self.__analyzer_class.analyze(self.get_car(), self.get_rules(), monitor_info)
            self.__waypoint_num = analyzer_info["new_waypoint"]
            plan_info = self.__planner_class.plan(analyzer_info, speed_limit, self.get_checks())
            self.__executor_class.execute(plan_info, self.get_car())
    
        except:
            raise

