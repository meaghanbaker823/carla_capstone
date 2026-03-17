from classes.MAPEStep import MAPEStep
from classes.angleConstraint import AngleConstraint

import math
import carla
import numpy as np
from classes.route_done import RouteDone
from classes.pedestrianRule import PedestrianRule
from classes.parkedRule import ParkedRule

"""
===========
Analyzer Class()

__init__(self) creates instance and initilizes attributes
    self.__old_parameters | (list) list of parameter history
    self.__new_parameters | (list) new parameters for this iteration
    self.__new_observations | (list) observations from this iteration
    self.__old_observations | (list) list of observation history

function(self) return type | description

===========
"""

class Analzyer(MAPEStep):
    def __init__(self):
        self.__old_parameters = []
        self.__new_parameters = []
        self.__new_observations = []
        self.__old_observations = []

    def get_new_observations(self):
        return self.__new_observations

    
    def add_old_observations(self, observations):
        self.__old_observations.append(observations)

    def angle_between(self, v1, v2):
        try:
            assert type(v1) == tuple and type(v2) == tuple
        except:
            raise
        return math.degrees(np.arctan2(v1[1], v1[0]) - np.arctan2(v2[1], v2[0]))

    # function to get angle between the car and target waypoint
    def get_angle(self, car, wp):
        try:
            assert type(car) == carla.libcarla.Vehicle
            assert type(wp) == carla.Waypoint
        except:
            raise

        vehicle_pos = car.get_transform()
        car_x = vehicle_pos.location.x
        car_y = vehicle_pos.location.y
        wp_x = wp.transform.location.x
        wp_y = wp.transform.location.y
        
        # vector to waypoint
        x = (wp_x - car_x)/((wp_y - car_y) ** 2 + (wp_x - car_x) ** 2) ** 0.5
        y = (wp_y - car_y)/((wp_y - car_y) ** 2 + (wp_x - car_x) ** 2) ** 0.5
        
        #car vector
        car_vector = vehicle_pos.get_forward_vector()
        degrees = self.angle_between((x, y),(car_vector.x, car_vector.y))

        corrected_deg = self.correct_angle(degrees)
        return corrected_deg
    
    def get_proper_angle(self, car,wp_idx,rte):
        try:
            assert type(car) == carla.libcarla.Vehicle
            assert type(wp_idx) == int
            assert type(rte) == list
        except:
            raise

        # create a list of angles to next 5 waypoints starting with current
        next_angle_list = []
        for i in range(10):
            if wp_idx + i * 3 < len(rte) - 1:
                next_angle_list.append(self.get_angle(car, rte[wp_idx + i*3][0]))
        idx = 0
        while idx < len(next_angle_list) - 2 and abs(next_angle_list[idx]) > 40:
            idx += 1
        try:
            return wp_idx + idx * 3, next_angle_list[idx]
        except:
            raise RouteDone("Route complete :)")
    
    
    def correct_angle(self, degrees):
        try:
            assert (-360 <= degrees <= 360)
        except:
            raise

        degree_constraint = AngleConstraint(-300, 300, 360)
        fixed_deg = degree_constraint.clamp(degrees)

        # limit steering to max angle 50 degrees
        steer_constraint = AngleConstraint(-50, 50, 0)
        steer_input = steer_constraint.max_steer(fixed_deg)

        try:
            assert (-50 <= steer_input <= 50)
        except:
            raise
        
        return steer_input

    # returns possible waypoint in another lane, or None if none available
    def check_lane_options(self, waypoint_num, route, lane_change):
        try:
            assert isinstance(waypoint_num, int)
            assert isinstance(lane_change, carla.libcarla.LaneChange)
        except:
            raise

        current_waypoint = route[waypoint_num]

        # how far ahead in other lane are we looking
        step_size = 5

        # favor lane changes into the right lane
        if lane_change == carla.libcarla.LaneChange.NONE:
            return None
        elif lane_change == carla.libcarla.LaneChange.Right or lane_change == carla.libcarla.LaneChange.Both:
            right_lane = current_waypoint.get_right_lane()

            return right_lane.next(step_size)[0]
        else:
            left_lane = current_waypoint.get_left_lane()

            return left_lane.previous(step_size)[0]

    def analyze(self, car, rules, detections):
        self.__old_parameters.append(self.__new_parameters)
        self.__new_parameters = [car, rules, detections]
        self.add_old_observations(self.__new_observations)
        self.__new_observations = {}

        """
        __new_observations =
            rules:              (*classes.rules.Rule)
            traffic_lights:     (classes.trafficLight.TrafficLight)
            open_lane:          (carla.libcarla.Waypoint | None)
            current_speed:      (int)
            new_waypoint:       (int)
            steering_angle:     (float)
        """
        self.__new_observations["r"] = 0
        
        self.__new_observations["rules"] = []
        self.__new_observations["traffic_lights"] = detections["traffic_lights"]

        for rule in rules:
            # if false, then rule not passed
            try:
                if not rule.rule_flag(detections["traffic_lights"]):
                    self.__new_observations["rules"].append(rule)
            except:
                raise

        for rule in self.__new_observations["rules"]:
            if(isinstance(rule, ParkedRule)):
                self.__new_observations["distance"] = rule.get_sensors()[0].get_detections()[-1][1]
                self.__new_observations["r"] = 1

        self.__new_observations["open_lane"] = self.check_lane_options(detections["current_waypoint_num"], 
                detections["route"], detections["lane_info"]["lane_change"])
                    
        # unit is kilometers/hr
        self.__new_observations["current_speed"] = round(3.6 * math.sqrt(detections["current_velocity"].x ** 2 + detections["current_velocity"].y ** 2 + detections["current_velocity"].z ** 2), 0)

        waypoint_num, steering_angle = self.get_proper_angle(car, detections["current_waypoint_num"], detections["route"])
      
        self.__new_observations["new_waypoint"] = waypoint_num
        self.__new_observations["steering_angle"] = steering_angle / 75

        # the information that the planner needs
        return self.__new_observations
    
    def notify(self):
        output = "The observations in this analyzer iteration are: "
        for i in self.get_new_observations():
            output += "Observation: " + i
        return output
