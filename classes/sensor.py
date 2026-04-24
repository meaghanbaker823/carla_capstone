class Sensor:
    """
    The superclass for all other sensors
    """   
    def __init__(self, relative_transform, parent_actor, blueprint, world):
        """
        Initializes the variables needed for the Sensor class
        \n\tINPUT(S):
        \n\tOUTPUT(S):
        """   
        self.__transform = relative_transform
        self.__parent = parent_actor
        self.__sensor = world.get_world().spawn_actor(blueprint, relative_transform, attach_to=self.__parent)

    def get_transform(self):
        """
        Getter for self.__transform
        \n\tINPUT(S):
        \n\tOUTPUT(S):
        """   
        return self.__transform
    
    def get_parent(self):
        """
        Getter for self.__parent
        \n\tINPUT(S):
        \n\tOUTPUT(S):
        """   
        return self.__parent
    
    def get_world(self):
        """
        Getter for self.__world
        \n\tINPUT(S):
        \n\tOUTPUT(S):
        """   
        return self.__world
    
    def get_blueprint(self):
        """
        Getter for self.__blueprint
        \n\tINPUT(S):
        \n\tOUTPUT(S):
        """   
        return self.__blueprint
    
    def get_sensor(self):
        """
        Getter for self.__sensor
        \n\tINPUT(S):
        \n\tOUTPUT(S):
        """   
        return self.__sensor
    
    def listen():
        """
        Placeholder for the listen callback functions of each sensor
        \n\tINPUT(S):
        \n\tOUTPUT(S):
        """   
        pass