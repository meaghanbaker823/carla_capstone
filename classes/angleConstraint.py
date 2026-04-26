class AngleConstraint():
    """
    The class for clamping and adjusting angles
    """
    def __init__(self, min, max, adjust):
        """
        \n\tINPUT(S): min: the minimum angle allowed,
                      max: the maximum angle allowed,
                      adjust: the maximum the angle can change
        \n\tOUTPUT(S): N/A
        """
        self.__min = min
        self.__max = max
        self.__adjust = adjust

    def clamp(self, degrees):
        """
        Clamps the angle to be adjusted by a set amount if it is outside the given maximum and minimum
        \n\tINPUT(S): Degrees: the angle to be adjusted
        \n\tOUTPUT(S): the adjusted angle
        """
        if degrees < self.__min:
            return degrees + self.__adjust
        elif degrees > self.__max:
            return degrees - self.__adjust

        return degrees
    
    def max_steer(self, degrees):
        """
        Adjusts the steering angle provided to be between the maximum and minimum inclusively
        \n\tINPUT(S): degrees: the proposed steering angle
        \n\tOUTPUT(S): the adjusted steering angle
        """
        if degrees < self.__min:
            return self.__min
        elif degrees > self.__max:
            return self.__max
        else:
            return degrees    