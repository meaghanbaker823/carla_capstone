from classes.MAPEStep import MAPEStep
from classes.CBF import CBF
from classes.route_done import RouteDone
from classes.angleConstraint import AngleConstraint
import carla
import math
import numpy as np


"""
===========
Planner Class()

__init__(self) creates instance and initilizes attributes
    self.__

function(self) return type | description

===========
"""

class Planner(MAPEStep):
    def __init__(self):
        self.__old_parameters = []
        self.__new_parameters = []
        self.__old_plans = []
        self.__new_plan = {"brake": None, "steering": None, "throttle": None}

    def get_new_plan(self):
        return self.__new_plan
    
    def angle_between(self, v1, v2):
        try:
            assert isinstance(v1, tuple) and isinstance(v2, tuple)
        except:
            raise

        vector_difference = np.arctan2(v1[1], v1[0]) - np.arctan2(v2[1], v2[0])

        while vector_difference > math.pi:
            vector_difference -= 2 * math.pi

        while vector_difference < -math.pi:
            vector_difference += 2 * math.pi
        return math.degrees(vector_difference)

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

    def plan(self, car, observations, DT, extra, u_nom, alpha, max_acc, max_brake, standard_distance):
        self.__old_parameters.append(self.__new_parameters)
        self.__new_parameters = [observations]
        self.__old_plans.append(self.__new_plan)
        route = observations["route"]
        inverse_flag = False

        self.__new_plan = {"brake": None, "steering": None, "throttle": None}

            # unit is kilometers/h
        observations["current_speed"] = round(3.6 * math.sqrt(observations["current_velocity"].x ** 2 + observations["current_velocity"].y ** 2 + observations["current_velocity"].z ** 2), 0)

        waypoint_num, steering_angle = self.get_proper_angle(car, observations["current_waypoint_num"], route, inverse_flag)
      
        self.__new_plan["new_waypoint"] = waypoint_num
        observations["steering_angle"] = steering_angle / 75
        print("Steering angle: ", observations["steering_angle"])


        if(observations["rules"] == 0):
            observations["distance"] = 2
        
        self.__new_plan["steering"] = observations["steering_angle"]
        
        speed = observations["current_speed"] / 3.6

        cbf = CBF(observations["r"], speed, observations["distance"])
        min_distance = cbf.calculate_min_distance(standard_distance, extra)
        h = cbf.calculate_safety_function(min_distance)
        allowable_a = cbf.calculate_allowable_distance(alpha, h, DT)
        new_speed = cbf.final_logic(u_nom, allowable_a, max_acc, max_brake)        

        print("Speed: ", speed)
        if new_speed > 0:
            self.__new_plan["throttle"] = new_speed
            self.__new_plan["brake"] = 0
        else:
            self.__new_plan["throttle"] = 0
            self.__new_plan["brake"] = 1
        
        return self.__new_plan
    
    def notify(self):
        return "The plan in this iteration is " + str(self.get_new_plan())
