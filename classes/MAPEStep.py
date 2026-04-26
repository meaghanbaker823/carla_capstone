class MAPEStep():
    """
    The superclass for each of the portions of the MAPE structure
    """
    def __init__(self):
        """
        Sets up the variables needed for the MAPEStep class
        \n\tINPUT(S):N/A
        \n\tOUTPUT(S): N/A
        """
        self.__old_steps = []
        self.__new_step = ""

    def notify(self):
        """
        The superclass method for notifying the MAPE Step of its actions
        \n\tINPUT(S): N/A
        \n\tOUTPUT(S): N/A
        """
        pass
