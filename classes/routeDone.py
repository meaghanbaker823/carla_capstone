class RouteDone(Exception):
    """
    The class to handle when a route is done
    """
    def __init__(self, message):
        """
        Sets up the variables needed for the RouteDone class
        \n\tINPUT(S):
        \n\tOUTPUT(S):
        """
        super().__init__(message)