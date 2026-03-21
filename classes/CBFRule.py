class CBFRule():
    def __init__(self, r, s, distance):
        self.__r = r
        self.__distance = distance + 15
        self.__s = s


    def calculate_min_distance(self, extra):
        return self.__distance + (self.__r * extra)
    
    def calculate_safety_function(self, min_distance):
        return self.__distance - min_distance
    
    def calculate_allowable_distance(self, alpha, h, DT):
        return ((alpha * h) - self.__s) / DT
    
    def get_u_nom(self):
        return 1

    def final_logic(self, u_nom, u_cbf, max_acc):
        return min(u_nom, u_cbf, max_acc)
    
    def rule_follow(self, traffic_lights, limit):
        return 1, 0, None
    def rule_flag(self, traffic_lights):
        return True


""" 
TODO 
    1.  Figure out how to tie into maintain_speed?
    2.  NEW "The system shall detect pedetrians within the crosswalk"
    3.  NEW "The system shall enforce a max vehicle speed near the crosswalk" 
    4.  NEW "The system shall comput an emergency braking commands when 
                pedestrian is in stopping distance"
    5.  NEW "The controller shall minimize abrupt changes in acceleration between cycles"
"""