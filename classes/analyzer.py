from classes.MAPEStep import MAPEStep
from classes.parkedRule import ParkedRule

import carla

class Analyzer(MAPEStep):
    """
    The A in the MAPE structure.  Takes in the detections from monitor and decides what should be reacted to.  
    If a car must swerve around another car, the waypoint adjustment is completed in this class.
    """
    def __init__(self, carla_world):
        """
        \n\tINPUT(S): Carla world object
        \n\tOUTPUT(S): N/A
        """
        self.__carla_world = carla_world
        self.__old_parameters = []
        self.__new_parameters = []
        self.__new_observations = []
        self.__old_observations = []

    def get_new_observations(self):
        """
        Getter method for the self.__new_observations
        \n\tINPUT(S): N/A
        \n\tOUTPUT(S): new_observations: the dictionary which is passed through the MAPE steps
        """
        return self.__new_observations
    
    def add_old_observations(self, observations):
        """
        Setter method for the self.__old_observations
        \n\tINPUT(S): Observations dictionary (the dictionary passed through the MAPE steps)
        \n\tOUTPUT(S): N/A
        """
        self.__old_observations.append(observations)

    def check_lane_options(self, waypoint_num, route, lane_change):
        """
        Checks the waypoint's lane options to decide which way to swerve
        \n\tINPUT(S): waypoint_num: an index of route
                      route: the set of waypoints for the route of the vehicle
                      lane_change: says if a lane change is possible
        \n\tOUTPUT(S): the new route to follow
        """
        try:
            assert isinstance(waypoint_num, int)
            assert isinstance(lane_change, carla.libcarla.LaneChange)
        except:
            raise

        new_route = route
        swerve_range = 44
        start = 11
        force_shift = True

        if(force_shift):
            for i in range(start, swerve_range):
                swerve_point = i + waypoint_num
                if(swerve_point == (len(new_route) - 1)):
                    break
                new_route[swerve_point] = self.swerve(route[swerve_point], "left")
            return new_route

        # favor lane changes into the right lane (currently left for implementation)
        if lane_change == carla.libcarla.LaneChange.NONE:
            return new_route
            
        if lane_change == carla.libcarla.LaneChange.Left or lane_change == carla.libcarla.LaneChange.Both:
            for i in range(start, swerve_range):
                swerve_point = i + waypoint_num
                if(swerve_point == (len(new_route) - 1)):
                    break
                new_route[swerve_point] = self.swerve(route[swerve_point], "left")
            return new_route
        else:
            for i in range(start, swerve_range):
                swerve_point = i + waypoint_num
                if(swerve_point == (len(new_route) - 1)):
                    break
                new_route[swerve_point] = self.swerve(route[swerve_point], "right")
            return new_route        
        

    def swerve(self, waypoint, direction):
        """
        Adjusts one waypoint to move twice in the direction of the vector chosen and returns the waypoint at that location
        \n\tINPUT(S): waypoint: a specific waypoint in the route
                      direction: which direction is the lane in
        \n\tOUTPUT(S): the new waypoint
        """
        w_x = waypoint[0].transform.location.x
        w_y = waypoint[0].transform.location.y
        w_z = waypoint[0].transform.location.z

        vector = waypoint[0].transform.rotation.get_right_vector()
        z = w_z

        if(direction == "right"):
            x = w_x + (2 * vector.x)
            y = w_y + (2 * vector.y)
        else:
            x = w_x - (2 * vector.x)
            y = w_y - (2 * vector.y)

        new_location = carla.Location(x, y, z)
        new_waypoint = self.__carla_world.get_map().get_waypoint(new_location, project_to_road=True, lane_type=carla.LaneType.Driving)

        return (new_waypoint, waypoint[1])

    def analyze(self, car, rules, detections, distance):
        """
        Creates observations to be sent to the planner based on information from the monitor.
        \n\tINPUT(S): car: the Vehicle object
                      rules: a list containing the various rule objects,
                      detections: the dictionary passed through the MAPE steps
                      distance: the standard following distance
        \n\tOUTPUT(S): the dictionary passed between MAPE steps
        """
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
        self.__new_observations["rules"] = 1
        self.__new_observations["traffic_lights"] = detections["traffic_lights"]
        self.__new_observations["distance"] = distance
        self.__new_observations["current_velocity"] = detections["current_velocity"]
        self.__new_observations["current_waypoint_num"] = detections["current_waypoint_num"]
        new_route = detections["route"]
        for rule in reversed(rules):

            # collision, pedestrian, traffic light, parkedRule
            try:
                if not rule.rule_flag(detections["traffic_lights"]):
                    self.__new_observations["rules"] = rule.rule_follow(detections["traffic_lights"])
                    if isinstance(rule, ParkedRule):
                        self.__new_observations["distance"] = rule.get_sensors()[0].get_detections()[-1][1]
                        self.__new_observations["r"] = 1

                        new_route = self.check_lane_options(detections["current_waypoint_num"], detections["route"], detections["lane_info"]["lane_change"])
    
            except:
                raise
        
        self.__new_observations["route"] = new_route

        # the information that the planner needs
        return self.__new_observations
    

    def notify(self):
        """
        Formats the information from this step in this cycle to be easily read in the MAPE Step
        \n\tINPUT(S):
        \n\tOUTPUT(S):
        """
        output = "The observations in this analyzer iteration are: "
        for i in self.get_new_observations():
            output += "Observation: " + i
            output += " "
        return output
