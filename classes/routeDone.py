class RouteDone(Exception):
    """
    The class to handle when a route is done
    """
    def __init__(self, message):
        """
        \n\tINPUT(S): message: the message to be displayed when error is thrown
        \n\tOUTPUT(S): throws this error
        """
        super().__init__(message)