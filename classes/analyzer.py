from classes.MAPEStep import MAPEStep
from classes.angleConstraint import AngleConstraint

import math
import carla
import numpy as np
from classes.route_done import RouteDone
from classes.pedestrianRule import PedestrianRule
from classes.collisionRule import CollisionRule
from classes.trafficRule import TrafficRule
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

class Analyzer(MAPEStep):
    def __init__(self, carla_world):
        self.__carla_world = carla_world
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
            assert isinstance(v1, tuple) and isinstance(v2, tuple)
        except:
            raise
        return math.degrees(np.arctan2(v1[1], v1[0]) - np.arctan2(v2[1], v2[0]))

    # function to get angle between the car and target waypoint
    def get_angle(self, car, wp):
        try:
            assert isinstance(car, carla.libcarla.Vehicle)
            assert isinstance(wp, carla.Waypoint)
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
    
    def get_proper_angle(self, car,wp_idx, rte, inverse_flag):
        try:
            assert isinstance(car, carla.libcarla.Vehicle)
            assert isinstance(wp_idx, int)
            assert isinstance(rte, list)
        except:
            raise
        if (inverse_flag):
            invert = -1
        else: 
            invert = 1 
        # create a list of angles to next 5 waypoints starting with current
        next_angle_list = []
        for i in range(10):
            if wp_idx + i * 3 < len(rte) - 1:
                next_angle_list.append(self.get_angle(car, rte[wp_idx + i*3][0]))
        idx = 0
        while idx < len(next_angle_list) - 2 and abs(next_angle_list[idx]) > 40:
            idx += 1
        try:
            return wp_idx + idx * 3, next_angle_list[idx] * invert
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
    def check_lane_options(self, waypoint_num, route, lane_change, slope):
        try:
            assert isinstance(waypoint_num, int)
            assert isinstance(lane_change, carla.libcarla.LaneChange)
        except:
            raise

        new_route = route
        swerve_range = 100

        # favor lane changes into the right lane (currently left for implementation)
        if lane_change == carla.libcarla.LaneChange.NONE:
            print("no lane_change")
            return new_route
        if lane_change == carla.libcarla.LaneChange.Left or lane_change == carla.libcarla.LaneChange.Both:
            for i in range(10, swerve_range):
                swerve_point = i + waypoint_num
                new_route[swerve_point] = self.swerve(route[swerve_point], "left", slope)
            return new_route
        else:
            print("else lane ")
            for i in range(10, swerve_range):
                swerve_point = i + waypoint_num
                new_route[swerve_point] = self.swerve(route[swerve_point], "right", slope)
            return new_route        
        

    def swerve(self, waypoint, direction, slope):

        # pass slope as a parameter generated from the change of y and change of x (or velocity/speed)
        # calculate the y intercept based on the current y and x 
        # shift the line, left or right, based on multiplying the shift by 1 or -1
        # shift the line over using y-intercept however much is needed to get into the next lane
            # rotation might be involved? so potentially +180

        x = waypoint[0].transform.location.x
        y = waypoint[0].transform.location.y
        print("OLD")
        print(waypoint[0].transform.rotation)
        # print(f"OLD: X: {x:.2f}, Y: {y:.2f}")

        y_int = y - (slope * x) - 25
        d = (x + (y - y_int) * slope) / 1 + slope**2

        new_x = 2 * d - x
        new_y = 2 * d * slope - y + 2 * y_int  
        # print(f"NEW: X: {new_x:.2f}, Y: {new_y:.2f}")

        new_location = carla.Location(new_x, new_y, waypoint[0].transform.location.z)

        new_waypoint = self.__carla_world.get_map().get_waypoint(new_location, project_to_road=True, lane_type=carla.LaneType.Driving)
        print("NEW")
        print(new_waypoint.transform.rotation)

        return (new_waypoint, waypoint[1])

    def analyze(self, car, rules, detections, distance):
        self.__old_parameters.append(self.__new_parameters)
        self.__new_parameters = [car, rules, detections]
        self.add_old_observations(self.__new_observations)
        self.__new_observations = {}
        inverse_flag = False

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
        self.__new_observations["rules"] = 1
        self.__new_observations["traffic_lights"] = detections["traffic_lights"]
        self.__new_observations["distance"] = distance
        new_route = detections["route"]
        for rule in reversed(rules):
            # if false, then rule not passed

            # collision, pedestrian, traffic light, parkedRule
            try:
                if not rule.rule_flag(detections["traffic_lights"]):
                    self.__new_observations["rules"] = rule.rule_follow(detections["traffic_lights"])
                    if isinstance(rule, ParkedRule):
                        self.__new_observations["distance"] = rule.get_sensors()[0].get_detections()[-1][1] + 15
                        self.__new_observations["r"] = 1
                        old_check = new_route[detections["current_waypoint_num"] + 10][0].transform.location

                        x1 = new_route[detections["current_waypoint_num"]][0].transform.location.x
                        y1 = new_route[detections["current_waypoint_num"]][0].transform.location.y
                        x2 = new_route[detections["current_waypoint_num"] + 30][0].transform.location.x
                        y2 = new_route[detections["current_waypoint_num"] + 30][0].transform.location.y
                        print(f"X1: {x1:.2f}, X2: {x2:.2f} Y1: {y1:.2f} Y2: {y2:.2f}")
                        slope = (x2 - x1) / (y2 - y1)
                        print("SLOPE: " + str(slope))

                        fix_route = self.check_lane_options(detections["current_waypoint_num"], detections["route"], detections["lane_info"]["lane_change"], slope)
                        if fix_route == new_route:
                            inverse_flag = True
                            for i in range(10, 39):
                                swerve_point = i + detections["current_waypoint_num"]
                                fix_route[swerve_point] = self.swerve(new_route[swerve_point], "right", slope)
                            new_route = fix_route
                            print(f"UNCHANGED: {old_check}")
                            print(f"CHANGED: {new_route[detections['current_waypoint_num'] + 10][0].transform.location}")
                            for waypoint in new_route:
                                self.__carla_world.debug.draw_string(waypoint[0].transform.location, 'O', draw_shadow = False, color = carla.Color(r = 255, g = 0, b = 255), life_time = 10000.0, persistent_lines = True)
    
    
            except:
                raise

        # unit is kilometers/hr
        self.__new_observations["current_speed"] = round(3.6 * math.sqrt(detections["current_velocity"].x ** 2 + detections["current_velocity"].y ** 2 + detections["current_velocity"].z ** 2), 0)

        waypoint_num, steering_angle = self.get_proper_angle(car, detections["current_waypoint_num"], new_route, inverse_flag)
      
        self.__new_observations["new_waypoint"] = waypoint_num
        self.__new_observations["steering_angle"] = steering_angle / 75

        # the information that the planner needs
        return self.__new_observations
    
    def notify(self):
        output = "The observations in this analyzer iteration are: "
        for i in self.get_new_observations():
            output += "Observation: " + i
            output += " "
        return output
