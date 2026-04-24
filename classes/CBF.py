class CBF():
    """
    The class which holds all of the values and functions necessary to complete the Control Barrier Functions
    """
    def __init__(self, r, s, distance):
        """
        Sets up the variables needed for the CBF function
        \n\tINPUT(S):
        \n\tOUTPUT(S):
        """
        self.__r = r
        self.__distance = distance
        self.__s = s

    def calculate_min_distance(self, extra, standard_distance):
        """
        Calculates the minimum distance from the standard following distance, occlusion risk, and the extra following distance
        \n\tINPUT(S):
        \n\tOUTPUT(S):
        """
        return standard_distance + (self.__r * extra)
    
    def calculate_safety_function(self, min_distance):
        """
        Calculates the safety function by subtracting the current distance by the minimum distance
        \n\tINPUT(S):
        \n\tOUTPUT(S):
        """
        return self.__distance - min_distance
    
    def calculate_allowable_distance(self, alpha, h, DT):
        """
        Calculates the u_cbf value using the equation provided by Professor Wuwei Shen
        \n\tINPUT(S):
        \n\tOUTPUT(S):
        """
        return (((alpha * h) - self.__s) / DT/1000)

    def final_logic(self, u_nom, u_cbf, max_acc, max_brake):
        """
        Clamps the final throttle/brake value to be within certain values
        \n\tINPUT(S):
        \n\tOUTPUT(S):
        """
        return max(min(u_nom, u_cbf, max_acc), -1 * (max_brake))