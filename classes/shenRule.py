import osqp

class Shen():
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
        pass

    def final_osqp_logic(self, u_nom, u_cbf, max_acc):
        return min(u_nom, u_cbf, max_acc)


""" trying to get (P, q, a, l, u_bound) 
    P is cost matrix (positive semidefinite matrix (maybe [1 0] [0 1]))
    q is cost vector (linear objective)
    a is the constraint matrix (linear constraints on input)
    l is the lower bound for the A matrix
    u_bound is the upper bound for the A matrix
"""