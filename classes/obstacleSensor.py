from classes.sensor import Sensor

import carla

class ObstacleSensor(Sensor):
    """
    Class that will handle obstacle detections
    """   
    def __init__(self, relative_transform, parent_actor, blueprint, world, blueprint_lib):
        """
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
            assert str(blueprint) == str(blueprint_lib.find('sensor.other.obstacle')) # blueprint must be obstacle detector
        except:
            raise
        super().__init__(relative_transform, parent_actor, blueprint, world)
        self.__other_actors = []
        self.__detections = []
    
    def get_detections(self):
        """
        Getter for self.__detections
        \n\tINPUT(S): N/A
        \n\tOUTPUT(S): returns a list of all of the detections
        """   
        return self.__detections
    
    def get_other_actors(self):
        """
        Getter for self.__other_actors
        \n\tINPUT(S): None
        \n\tOUTPUT(S): returns a list of the obstacles
        """   
        return self.__other_actors
    
    def delete_old_detection(self):
        """
        Removes the first item from the detections and other actors list
        \n\tINPUT(S): N/A
        \n\tOUTPUT(S): N/A
        """   
        detections_len = len(self.__detections)
        oactorer_len = len(self.__other_actors)

        self.__detections.pop(0)
        self.__other_actors.pop(0)

         # post condition
        try:
            assert len(self.__detections) == detections_len - 1
            assert len(self.__other_actors) == oactorer_len - 1
        except:
            raise
          
    def obstacle_detect(self, event):
        """
        Adds the obstacle detection to the list
        \n\tINPUT(S): event: the carla ObstacleEvent 
        \n\tOUTPUT(S): N/A
        """   
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

    def listen(self):
        """
        Sets up the listener for the obstacle detector with obstacle_detect as the callback function
        \n\tINPUT(S): N/A
        \n\tOUTPUT(S): N/A
        """   
        self.get_sensor().listen(lambda event: self.obstacle_detect(event))
