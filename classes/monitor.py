from classes.MAPEStep import MAPEStep
from classes.trafficLight import TrafficLight
from classes.obstacleSensor import ObstacleSensor
from classes.collisionSensor import CollisionSensor

import carla

"""
===========
Monitor Class(MAPEStep)

__init__(self) creates instance and initilizes attributes
    self.__

function(self) return type | description

===========
"""

class Monitor(MAPEStep):
    def __init__(self, transform, car, blueprint, world, blueprint_lib, actors, route):
        self.__old_parameters = []
        self.__new_parameter = None
        self.__old_detections = []
        self.__new_detections = {}
        self.__sensors = []
        self.__car = car
        self.__actors = actors
        self.__route = route

        self.add_sensors(ObstacleSensor(transform[0], car, blueprint[0], world, blueprint_lib))
        self.add_sensors(CollisionSensor(transform[1], car, blueprint[1], world, blueprint_lib))

        for sensor in self.__sensors:
            sensor.listen()

    def add_sensors(self, sensor):
        self.__sensors.append(sensor)

    def get_actors(self):
        return self.__actors

    def get_sensors(self):
        return self.__sensors
    
    def get_new_detections(self):
        return self.__new_detections
    
    def add_old_detections(self, detections):
        self.__old_detections.append(detections)

    def advance_waypoint(self, current_waypoint_num):
        try:
            assert type(self.__car) == carla.libcarla.Vehicle
        except:
            raise
        
        condition1 = current_waypoint_num < len(self.__route)
        condition2 = self.__car.get_transform().location.distance(self.__route[current_waypoint_num][0].transform.location) < 5

        while current_waypoint_num < len(self.__route) and self.__car.get_transform().location.distance(self.__route[current_waypoint_num][0].transform.location)<5:
            print(current_waypoint_num)
            current_waypoint_num += 1
        return current_waypoint_num

    def monitor(self, current_waypoint_num):
        self.__old_parameters.append(self.__new_parameter)
        self.__new_parameter = current_waypoint_num

        self.add_old_detections(self.__new_detections)
        self.__new_detections = {}

        # unit is m/s (a 3d vector)
        self.__new_detections["current_velocity"] = self.__car.get_velocity()
        self.__new_detections["traffic_lights"] = TrafficLight(self.get_actors())

        self.__new_detections["current_waypoint_num"] = self.advance_waypoint(current_waypoint_num)
        # print(f"current waypoint: {self.__new_detections['current_waypoint_num']}")
        # print(len(self.__route))
        self.__new_detections["route"] = self.__route
    
        #return the info that the analyzer needs
        return self.__new_detections
    
    def notify(self):
        output = "The detections in this monitor iteration are: "
        for i in self.get_new_detections():
            output += "Detection: " + i
        return output