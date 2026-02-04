import carla
import traceback
import math
import numpy as np

from classes.globalRoutePlanner import GlobalRoutePlanner
from classes.angleConstraint import AngleConstraint


"""
===========
Navigation Class

__init__(self, start, destination, world) creates instance and initilizes attributes
    self.__start = the inital location of the vechicle
    self.__destination = the coorinates the car wants to go to 
    self.__global_route_planner = the global route planner object
    self.__route = set of waypoints (the route)
   

getters for each of these attributes

lane_invasion(self, event) | adds collision (parent actor which invaded lane, line markings which were crossed) to self.__lane_invasions

listen(self) | retreives data from sensor and calls the lane_invasion method
===========
"""

class Navigation():
    def __init__(self, start, destination, world):
        try:
            assert type(start) == carla.libcarla.Transform
            assert type(destination) == carla.libcarla.Transform
            assert type(world) == carla.libcarla.World 
        except:
            print(traceback.format_exc())

        self.__start = start.location
        self.__destiniation = destination.location
        self.__global_route_planner = GlobalRoutePlanner(world.get_map(), 1)
        self.__route = self.__global_route_planner.trace_route(self.__start, self.__destiniation)
        self.__current_waypoint_num = 5
        self.__current_waypoint = self.__route[self.__current_waypoint_num]

        # visualizing waypoints
        for waypoint in self.__route:
            world.debug.draw_string(waypoint[0].transform.location, '^', draw_shadow=False,
            color=carla.Color(r=0, g=0, b=255), life_time=90.0,
            persistent_lines=True)
    
    def get_start(self):
        return self.__start
    def get_destination(self):
        return self.__destiniation
    def get_global_route_planner(self):
        return self.__global_route_planner
    def get_route(self):
        return self.__route
    def get_cur_waypoint(self):
        return self.__current_waypoint
    def get_cur_waypoint_num(self):
        return self.__current_waypoint_num
    def set_cur_waypoint(self, num):
        self.__current_waypoint = self.__route[num]
        self.__current_waypoint_num = num
    def advance_waypoint(self, car):
        try:
            assert type(car) == carla.libcarla.Vehicle
        except:
            print(traceback.format_exc())

        while self.__current_waypoint_num < len(self.__route) and car.get_transform().location.distance(self.__route[self.__current_waypoint_num][0].transform.location)<5:
            self.__current_waypoint_num +=1
       
    
    # angle between two vectors
    def angle_between(self, v1, v2):
        try:
            assert type(v1) == tuple and type(v2) == tuple
        except:
            print(traceback.format_exc())
        return math.degrees(np.arctan2(v1[1], v1[0]) - np.arctan2(v2[1], v2[0]))

    # function to get angle between the car and target waypoint
    def get_angle(self,car,wp):
        try:
            assert type(car) == carla.libcarla.Vehicle
            assert type(wp) == carla.Waypoint
        except:
            print(traceback.format_exc())

        vehicle_pos = car.get_transform()
        car_x = vehicle_pos.location.x
        car_y = vehicle_pos.location.y
        wp_x = wp.transform.location.x
        wp_y = wp.transform.location.y
        
        # vector to waypoint
        x = (wp_x - car_x)/((wp_y - car_y)**2 + (wp_x - car_x)**2)**0.5
        y = (wp_y - car_y)/((wp_y - car_y)**2 + (wp_x - car_x)**2)**0.5
        
        #car vector
        car_vector = vehicle_pos.get_forward_vector()
        degrees = self.angle_between((x,y),(car_vector.x,car_vector.y))

        corrected_deg = self.correct_angle(degrees)
        return corrected_deg
    
    def get_proper_angle(self, car,wp_idx,rte):
        try:
            assert type(car) == carla.libcarla.Vehicle
            assert type(wp_idx) == int
            assert type(rte) == list
        except:
            print(traceback.format_exc())

        # create a list of angles to next 5 waypoints starting with current
        next_angle_list = []
        for i in range(10):
            if wp_idx + i*3 <len(rte)-1:
                next_angle_list.append(self.get_angle(car,rte[wp_idx + i*3][0]))
        idx = 0
        while idx<len(next_angle_list)-2 and abs(next_angle_list[idx])>40:
            idx +=1
        return wp_idx+idx*3,next_angle_list[idx]
    
    def correct_angle(self, degrees):
        try:
            assert (-360 <= degrees <= 360)
        except:
            print(traceback.format_exc())

        degree_constraint = AngleConstraint(-300, 300, 360)
        fixed_deg = degree_constraint.clamp(degrees)

        # limit steering to max angle 50 degrees
        steer_constraint = AngleConstraint(-50, 50, 0)
        steer_input = steer_constraint.max_steer(fixed_deg)

        try:
            assert (-50<= steer_input <= 50)
        except:
            print(traceback.format_exc())
        
        return steer_input