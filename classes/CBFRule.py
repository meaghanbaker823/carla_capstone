class CBFRule():
    def __init__(self, r, s, distance):
        self.__r = r
        self.__distance = distance
        self.__s = s


    def calculate_min_distance(self, extra, standard_distance):
        return standard_distance + (self.__r * extra)
    
    def calculate_safety_function(self, min_distance):
        return self.__distance - min_distance
    
    def calculate_allowable_distance(self, alpha, h, DT):
        return ((alpha * h) - self.__s) / DT

    def final_logic(self, u_nom, u_cbf, max_acc):
        return min(u_nom, u_cbf, max_acc)
    
    def rule_follow(self, traffic_lights, limit):
        return 1, 0, None
    
    def rule_flag(self, traffic_lights):
        return True


""" 
TODO 
    3.  NEW "The system shall enforce a max vehicle speed near the crosswalk"
    4.  NEW "The system shall compute an emergency braking commands when pedestrian is in stopping distance"
    5.  NEW "The controller shall minimize abrupt changes in acceleration between cycles"
"""