class Sensor:
    """
    The superclass for all other sensors. Subclass sensors use polymorphism to modify
    the functions provided here.
    """   
    def __init__(self, relative_transform, parent_actor, blueprint, world):
        """
        \n\tINPUT(S): relative_transform: the transform object in relation to the parent actor,
                      parent_actor: the carla actor the sensor will be attached to,
                      blueprint: the blueprint for this sensor,
                      world: the world object,
                      blueprint_lib: the carla blueprint library
        \n\tOUTPUT(S): N/A
        """   
        self.__transform = relative_transform
        self.__parent = parent_actor
        self.__sensor = world.get_world().spawn_actor(blueprint, relative_transform, attach_to=self.__parent)

    def get_transform(self):
        """
        Getter for self.__transform
        \n\tINPUT(S): N/A
        \n\tOUTPUT(S): the relative transform of the sensor to the vehicle
        """   
        return self.__transform
    
    def get_parent(self):
        """
        Getter for self.__parent
        \n\tINPUT(S): N/A
        \n\tOUTPUT(S): the parent actor of the sensor (the actor the sensor is attached to)
        """   
        return self.__parent
    
    def get_world(self):
        """
        Getter for self.__world
        \n\tINPUT(S): N/A
        \n\tOUTPUT(S): the carla world
        """   
        return self.__world
    
    def get_blueprint(self):
        """
        Getter for self.__blueprint
        \n\tINPUT(S): N/A
        \n\tOUTPUT(S): the blueprint for the sensor
        """   
        return self.__blueprint
    
    def get_sensor(self):
        """
        Getter for self.__sensor
        \n\tINPUT(S): N/A
        \n\tOUTPUT(S): returns the carla sensor object
        """   
        return self.__sensor
    
    def listen():
        """
        Placeholder for the listen callback functions of each sensor
        \n\tINPUT(S): N/A
        \n\tOUTPUT(S): N/A
        """   
        pass