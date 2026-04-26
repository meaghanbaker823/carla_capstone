from classes.globalRoutePlanner import GlobalRoutePlanner
from classes.monitor import Monitor
from classes.analyzer import Analyzer
from classes.planner import Planner
from classes.executor import Executor

import random

class Vehicle: 
    """
    The class that controls the vehicle
    """
    def __init__(self, blueprint_lib, carla_world, spawn, destination, transform, blueprint, world, DT, extra, u_nom, alpha, max_acc, max_brake, distance, standard_distance):
        """
        \n\tINPUT(S): blueprint_lib: the carla blueprint library,
                      carla_world: the carla world,
                      spawn: the spawn point of the vehicle,
                      destination: the destination of the vehicle,
                      transform: a list of relative transform for the sensors,
                      blueprint: a list of blueprints for the sensors,
                      world: the world object,
                      DT: the tick speed,
                      extra: a multipler for CBF,
                      u_nom: a constant for CBF,
                      alpha: a constant for CBF,
                      max_acc: the maximum acceleration,
                      max_brake: the maximum brake value,
                      distance: the base distance for vehicle
                      standard_distance: the standard following distance for the vehicle
        \n\tOUTPUT(S): N/A
        """
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
        self.mape_init(transform, blueprint, blueprint_lib)

    def mape_init(self, transform, blueprint, blueprint_lib):
        """
        Sets up all of the MAPE classes and completes one cycle
        \n\tINPUT(S): transform: a list of relative transform for the sensors,
                      blueprint: a list of blueprints for the sensors,
                      blueprint_lib: the carla blueprint library,
        \n\tOUTPUT(S): N/A
        """
        try:
            self.__monitor_class = Monitor(transform, self.__car, blueprint, self.__world, blueprint_lib, self.__actors, self.__route)
            self.__analyzer_class = Analyzer(self.__carla_world)
            self.__planner_class = Planner()
            self.__executor_class = Executor()

            monitor_info = self.__monitor_class.monitor(self.__waypoint_num)

            self.__waypoint_num = monitor_info["current_waypoint_num"]

            analyzer_info = self.__analyzer_class.analyze(self.get_car(), self.get_rules(), monitor_info, self.__distance)

            plan_info = self.__planner_class.plan(self.get_car(), analyzer_info, self.__DT, self.__extra, self.__u_nom, self.__alpha, self.__max_acc, self.__max_brake, self.__standard_distance)
            self.__waypoint_num = plan_info["new_waypoint"]

            self.__executor_class.execute(plan_info, self.get_car())
        except:
            raise
    
    def get_rules(self):
        """
        Getter for self.__rules
        \n\tINPUT(S): N/A
        \n\tOUTPUT(S): the list of Rule objects the vehicle abides by
        """
        return self.__rules
    
    def set_rules(self, rules):
        """
        Setter for self.__rules
        \n\tINPUT(S): rules: a list of Rule objects the vehicle abides by
        \n\tOUTPUT(S): N/A
        """
        self.__rules = rules

    def get_sensors(self):
        """
        Getter for the sensors
        \n\tINPUT(S): N/A
        \n\tOUTPUT(S): returns the list of sensors attached to the vehicle
        """
        return self.__monitor_class.get_sensors()
  
    def get_car(self):
        """
        Getter for self.__car
        \n\tINPUT(S): N/A
        \n\tOUTPUT(S): returns the carla vehicle object
        """
        return self.__car
    
    def get_actors(self):
        """
        Getter for self.__actors
        \n\tINPUT(S): N/A
        \n\tOUTPUT(S): The carla actors list
        """
        return self.__actors

    def mape_drive(self):  
        """
        Walks through the MAPE structure
        \n\tINPUT(S): N/A
        \n\tOUTPUT(S): N/A
        """   
        try:
            monitor_info = self.__monitor_class.monitor(self.__waypoint_num)

            self.__waypoint_num = monitor_info["current_waypoint_num"]

            analyzer_info = self.__analyzer_class.analyze(self.get_car(), self.get_rules(), monitor_info, self.__distance)

            plan_info = self.__planner_class.plan(self.get_car(), analyzer_info, self.__DT, self.__extra, self.__u_nom, self.__alpha, self.__max_acc, self.__max_brake, self.__standard_distance)
            self.__waypoint_num = plan_info["new_waypoint"]

            self.__executor_class.execute(plan_info, self.get_car())
    
        except:
            raise

