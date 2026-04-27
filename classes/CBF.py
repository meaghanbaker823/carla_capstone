class CBF():
    """
    The class which holds all of the values and functions necessary to complete the Control Barrier Functions
    """
    def __init__(self, r, s, distance):
        """
        \n\tINPUT(S): r: occulusion risk,
                      s: current speed of the vehicle,
                      distance: the standard following distance
        \n\tOUTPUT(S): N/A
        """
        self.__r = r
        self.__distance = distance
        self.__s = s

    def calculate_min_distance(self, extra, standard_distance):
        """
        Calculates the minimum distance from the standard following distance, occlusion risk, and 
        the extra following distance. The occulusion risk is always 0 or 1, efficively acting as a
        boolean variable for our extra distance.
        \n\tINPUT(S): extra: multiplier if there is occulsion risk,
                      standard_distance: the standard following distance
        \n\tOUTPUT(S): returns the minimum distance
        """
        return standard_distance + (self.__r * extra)
    
    def calculate_safety_function(self, min_distance):
        """
        Calculates the safety function by subtracting the current distance by the minimum distance
        that is calculated in calculate_min_distance
        \n\tINPUT(S): min_distance: the distance returned from calculate_min_distance (the minimum following distance)
        \n\tOUTPUT(S): the safe following distance
        """
        return self.__distance - min_distance
    
    def calculate_allowable_distance(self, alpha, h, DT):
        """
        Calculates the u_cbf value using the equation provided by Professor Wuwei Shen. This serves
        as the throttle value of the vehicle in most scenarios. The value is divided by 1000 to
        make the output a decimal.
        \n\tINPUT(S): alpha: constant provided
                      h: distance calculated by calculate_safety_function, 
                      DT: tick speed
        \n\tOUTPUT(S): returns the allowable distance
        """
        return (((alpha * h) - self.__s) / DT/1000)

    def final_logic(self, u_nom, u_cbf, max_acc, max_brake):
        """
        Clamps the final throttle/brake value to be within certain values.
        \n\tINPUT(S): u_nom: constant given, 
                      u_cbf: allowable distance,
                      max_acc: the maximum acceleration, 
                      max_brake: the maximum brake
        \n\tOUTPUT(S): the throttle the vehicle
        """
        return max(min(u_nom, u_cbf, max_acc), -1 * (max_brake))