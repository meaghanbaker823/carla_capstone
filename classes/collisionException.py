class CollisionErr(Exception):
    """
    The class to handle when a collision occurs
    """
    def __init__(self, message):
        """
        Sets up the variables needed for the CollisionErr class
        \n\tINPUT(S):
        \n\tOUTPUT(S):
        """
        super().__init__(message)