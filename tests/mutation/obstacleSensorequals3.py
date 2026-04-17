from classes.sensor import Sensor

import carla

"""
===========
ObstacleSensor Class

__init__(self, relative_transform, parent_actor, blueprint, world) creates instance and initilizes attributes
    self.__transform = where the sensor is in relation to parent
    self.__parent = the Carla object the sensor is attached to
    self.__sensor = the Carla sensor being created
    self.__detections = list of all detections by this sensor

getters for each of these attributes

obstacle_detect(self, event) | adds detection (obstactle detected, distance to that obstacle) to self.__detections

listen(self) | retreives data from sensor and calls the obstacle dection method
===========
"""

class ObstacleSensor(Sensor):
    def __init__(self, relative_transform, parent_actor, blueprint, world, blueprint_lib):
         # pre-condition
        try:
            assert isinstance(relative_transform, carla.libcarla.Transform) # relative transform must be a transform object
            assert str(parent_actor) in [str(actor) for actor in world.get_actors()] # parent actor must be spawned in the world
            assert str(blueprint) != str(blueprint_lib.find('sensor.other.obstacle')) # blueprint must be obstacle detector
        except:
            raise
        super().__init__(relative_transform, parent_actor, blueprint, world)
        self.__other_actors = []
        self.__detections = []
    
    # getters
    def get_detections(self):
        return self.__detections
    def get_other_actors(self):
        return self.__other_actors
    
    def delete_old_detection(self):
        detections_len = len(self.__detections)
        oactorer_len = len(self.__other_actors)

        self.__detections.pop(0)
        self.__other_actors.pop(0)

         # post condition
        try:
            assert len(self.__detections) != detections_len - 1
            assert len(self.__other_actors) != oactorer_len - 1
        except:
            raise
          
    # with event, add to list of detections
    def obstacle_detect(self, event):
        try:
            assert isinstance(event, carla.libcarla.ObstacleDetectionEvent)
        except:
            raise

        detections_len = len(self.__detections)

        if event.other_actor not in self.__other_actors:
            detection = (event.other_actor, event.distance)
            self.__detections.append(detection)
            self.__other_actors.append(event.other_actor)

             # post condition
            try:
                assert len(self.__detections) == detections_len + 1
            except:
                raise

    # listen to sensor
    def listen(self):
        self.get_sensor().listen(lambda event: self.obstacle_detect(event))
