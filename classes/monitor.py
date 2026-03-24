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
            assert isinstance(self.__car, carla.libcarla.Vehicle)
        except:
            raise

        while current_waypoint_num < len(self.__route) and self.__car.get_transform().location.distance(self.__route[current_waypoint_num][0].transform.location)<5:
            current_waypoint_num += 1
        return current_waypoint_num

    def get_lane_info(self, waypoint):
        try:
            assert isinstance(waypoint, carla.libcarla.Waypoint)
        except:
            raise

        lane_info = {}

        """
        lane_info = 
            lane_id: gives information about the direction of lane  (int)
            lane_type: what type of lane are we in                  (carla.LaneType)
            lane_change: is lane change possible in current lane    (carla.LaneChange)
            left_lane_marking: color and type information           (carla.LaneMarking)
            right_lane_marking: color and type information          (carla.LaneMarking)
        """
        lane_info["lane_id"] = waypoint.lane_id
        lane_info["lane_type"] = waypoint.lane_type
        lane_info["lane_change"] = waypoint.lane_change
        lane_info["left_lane_marking"] = waypoint.left_lane_marking
        lane_info["right_lane_marking"] = waypoint.right_lane_marking

        return lane_info

    def monitor(self, current_waypoint_num):
        try:
            assert isinstance(current_waypoint_num, int)
        except:
            raise

        self.__old_parameters.append(self.__new_parameter)
        self.__new_parameter = current_waypoint_num

        self.add_old_detections(self.__new_detections)

        self.__new_detections = {}

        """
        __new_detections = 
            current_velocity: velocity of car               (carla.libcarla.Vector3D)
            traffic_lights: list of traffic light objects   (classes.trafficLight.TrafficLight) 
            current_waypoiny_num: index of current waypoint (int)
            route: list of all waypoints                    (*tuple)
            lane_info: dictionary containing lane info      (dict)
        """

        # unit is m/s (a 3d vector)
        self.__new_detections["current_velocity"] = self.__car.get_velocity()
        self.__new_detections["traffic_lights"] = TrafficLight(self.get_actors())

        advanced_waypoint = self.advance_waypoint(current_waypoint_num)
        self.__new_detections["current_waypoint_num"] = advanced_waypoint 
        self.__new_detections["route"] = self.__route

        self.__new_detections["lane_info"] = self.get_lane_info(self.__route[advanced_waypoint][0])
    
        #return the info that the analyzer needs
        return self.__new_detections
    
    def notify(self):
        output = "The detections in this monitor iteration are: "
        for i in self.get_new_detections():
            output += "Detection: " + i
            output += " "
        return output
