from classes.sensor import Sensor

import carla

"""
===========
CollisionSensor Class

__init__(self, relative_transform, parent_actor, blueprint, world) creates instance and initilizes attributes
    self.__transform = where the sensor is in relation to parent
    self.__parent = the Carla object the sensor is attached to
    self.__sensor = the Carla sensor being created
    self.__collisions = list of all collisions by this sensor

getters for each of these attributes

collision_detect(self, event) | adds collision (parent actor, other actor in collision, intensity of impact) to self.__collisions

listen(self) | retreives data from sensor and calls the collision_detect method
===========
"""

class CollisionSensor(Sensor):
    def __init__(self, relative_transform, parent_actor, blueprint, world, blueprint_lib):
         # pre-condition
        try:
            assert isinstance(relative_transform, carla.libcarla.Transform) # relative transform must be a transform object
            assert str(parent_actor) in [str(actor) for actor in world.get_actors()] # parent actor must be spawned in the world
            assert str(blueprint) == str(blueprint_lib.find('sensor.other.collision')) # blueprint must be obstacle detector
        except:
            raise


        super().__init__(relative_transform, parent_actor, blueprint, world)
        self.__collisions = []
        
    
    # getters
    def get_collisions(self):
        return self.__collisions
    
    # with event, add to list of detections
    def collision_detect(self, event):
        # try:
        #     assert type(event) == carla.libcarla.Collision.Event
        # except:
        #     print(traceback.format_exc())

        collisions_len = len(self.__collisions)
        # other impulse is a change in momentum - indicates magnitute and direction in global coordinates
        collision = (event.actor, event.other_actor, event.normal_impulse)
        self.__collisions.append(collision)

        # post condition
        try:
            assert len(self.__collisions) == collisions_len - 1
        except:
            raise
        
    # listen to sensor
    def listen(self):
        self.get_sensor().listen(lambda event: self.collision_detect(event))
