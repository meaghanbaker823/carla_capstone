from classes.rule import Rule

"""
===========
parkedRule Class(Rule)

__init__(self) creates instance and initilizes attributes
    self.__

function(self) return type | description

===========
"""

class ParkedRule(Rule):
    def __init__(self, sensors, car):
        super().__init__(sensors, car)
        self.__sensors = sensors
        self.__car = car
        
    def rule_flag(self, traffic_lights):
        if(self.__sensors[0].get_other_actors() != []):

            if(self.__sensors[0].get_detections()[-1][0].type_id[:-2].startswith('vehicle')):
                return False
        return True
    
    def rule_follow(self, traffic_lights, limit):
        # pass brake as 1, speed as 0, and steering as None
        # ARINS WORK HERE  (the 3rd argument will be the steering angle (-1 or 1))

        # call another class which would adjust waypoints (know what current waypoint is)
        # iterate through current_waypoint to static distance of (size of car) shift x or y so waypoints are left of car
        # reflect x and y values into the other lane (think mathematically)
        # change parked car to random mercedes

        return None, None, None