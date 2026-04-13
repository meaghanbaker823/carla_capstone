from classes.globalRoutePlanner import GlobalRoutePlanner
from classes.monitor import Monitor
from classes.analyzer import Analyzer
from classes.planner import Planner
from classes.executor import Executor
import matplotlib.pyplot as plt


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
    def __init__(self, blueprint_lib, carla_world, spawn, destination, transform, blueprint, world, DT, extra, u_nom, alpha, max_acc, max_brake, distance, standard_distance):
        self.__car = carla_world.spawn_actor(random.choice(blueprint_lib.filter('vehicle.bmw.*')), spawn)
        self.__actors = carla_world.get_actors()
        self.__world = world
        self.__carla_world = carla_world
        self.__rules = []
        self.__waypoint_num = 5
        self.__DT = DT
        self.__extra = extra
        self.__u_nom = u_nom
        self.__alpha = alpha
        self.__max_acc = max_acc
        self.__max_brake = max_brake
        self.__distance = distance
        self.__standard_distance = standard_distance
        self.__global_route_planner = GlobalRoutePlanner(self.__carla_world.get_map(), 1)
        self.__route = self.__global_route_planner.trace_route(spawn.location, destination.location)
        self.__route = self.__global_route_planner.trace_route(spawn.location, destination.location)
        self.mape_init(transform, blueprint, blueprint_lib)
        self.draw_route(self.__route)

    def mape_init(self, transform, blueprint, blueprint_lib):
        try:
            self.__monitor_class = Monitor(transform, self.__car, blueprint, self.__world, blueprint_lib, self.__actors, self.__route)
            self.__analyzer_class = Analyzer(self.__carla_world)
            self.__planner_class = Planner()
            self.__executor_class = Executor()

            monitor_info = self.__monitor_class.monitor(self.__waypoint_num)

            self.__waypoint_num = monitor_info["current_waypoint_num"]

            analyzer_info = self.__analyzer_class.analyze(self.get_car(), self.get_rules(), monitor_info, self.__distance)
            self.__waypoint_num = analyzer_info["new_waypoint"]

            plan_info = self.__planner_class.plan(analyzer_info, self.__DT, self.__extra, self.__u_nom, self.__alpha, self.__max_acc, self.__max_brake, self.__standard_distance)

            self.__executor_class.execute(plan_info, self.get_car())
        except:
            raise

    def draw_route(self, route):
        for waypoint in route:
            self.__carla_world.debug.draw_string(waypoint[0].transform.location, '^', draw_shadow = False, color = carla.Color(r = 0, g = 0, b = 255), life_time = 5.0, persistent_lines = True)
    
    def get_rules(self):
        return self.__rules
    
    def set_rules(self, rules):
        self.__rules = rules

    def get_sensors(self):
        return self.__monitor_class.get_sensors()
  
    def get_car(self):
        return self.__car
    
    def get_actors(self):
        return self.__actors

    def mape_drive(self):     
        try:
            monitor_info = self.__monitor_class.monitor(self.__waypoint_num)

            self.__waypoint_num = monitor_info["current_waypoint_num"]

            analyzer_info = self.__analyzer_class.analyze(self.get_car(), self.get_rules(), monitor_info, self.__distance)
            self.__waypoint_num = analyzer_info["new_waypoint"]

            plan_info = self.__planner_class.plan(analyzer_info, self.__DT, self.__extra, self.__u_nom, self.__alpha, self.__max_acc, self.__max_brake, self.__standard_distance)

            self.__executor_class.execute(plan_info, self.get_car())
    
        except:
            raise

