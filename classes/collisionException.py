class CollisionErr(Exception):
    """
    The class to handle when a collision occurs
    """
    def __init__(self, message):
        """
        Sets up the variables needed for the CollisionErr class
        \n\tINPUT(S): mmessage: the message to be displayed when this exception is thrown
        \n\tOUTPUT(S): this error gets thrown
        """
        super().__init__(message)