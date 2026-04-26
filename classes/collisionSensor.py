from classes.sensor import Sensor

import carla

class CollisionSensor(Sensor):
    """
    The class for the sensor which reacts to collisions
    """   
    def __init__(self, relative_transform, parent_actor, blueprint, world, blueprint_lib):
        """
        Initializes the variables needed for the CollisionSensor class
        \n\tINPUT(S): relative_transform: the transform object in relation to the parent actor,
                      parent_actor: the carla actor the sensor will be attached to,
                      blueprint: the blueprint for this sensor,
                      world: the world object,
                      blueprint_lib: the carla blueprint library
        \n\tOUTPUT(S): N/A
        """ 
        
        # pre-condition
        try:
            assert isinstance(relative_transform, carla.libcarla.Transform) # relative transform must be a transform object
            assert str(parent_actor) in [str(actor) for actor in world.get_actors()] # parent actor must be spawned in the world
            assert str(blueprint) == str(blueprint_lib.find('sensor.other.collision')) # blueprint must be obstacle detector
        except:
            raise


        super().__init__(relative_transform, parent_actor, blueprint, world)
        self.__collisions = []
        
    
    def get_collisions(self):
        """
        Getter for self.__collisions
        \n\tINPUT(S): N/A
        \n\tOUTPUT(S): returns the list of collisions for the vehicle
        """   
        return self.__collisions
    
    def collision_detect(self, event):
        """
        Adds the collision to the collision list when a collision is detected
        \n\tINPUT(S): event: the carla CollisionEvent 
        \n\tOUTPUT(S): N/A
        """   
        collisions_len = len(self.__collisions)
        collision = (event.actor, event.other_actor, event.normal_impulse)
        self.__collisions.append(collision)

        # post condition
        try:
            assert len(self.__collisions) == collisions_len + 1
        except:
            raise
        
    def listen(self):
        """
        Starts the sensor to list with the collision_detect as it's callback function
        \n\tINPUT(S): N/A
        \n\tOUTPUT(S): N/A
        """   
        self.get_sensor().listen(lambda event: self.collision_detect(event))
