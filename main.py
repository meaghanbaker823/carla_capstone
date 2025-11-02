# imports
import carla
import os
import random
import time
import numpy as np
import cv2


#MAKE NEW CLASS FOR VEHICLES (VEHICLE WILL CALL THIS CLASS WHEN STARTING THE CAR MOVING AND IT WILL BE CALLED BY OUR LOOP (WHICH IS ALSO PROLLY A CLASS))
#EACH TYPE OF CHECKING OR MOVING SHOULD BE A CLASS (EACH SECTION OF FUNCTIONAL REQUIREMENTS ALMOST CAN BE A CLASS)

#SHOULD MAKE A CLASS THAT CHECKS FOR KEYBOARD INTERRUPT AND CALLS ANOTHER CLASS TO END THE PROGRAM
#THIS CLASS WOULD USE VEHICLES AND SENSORS TO UTILIZE VEHICLE FUNCTIONS TO CORRECTLY MOVE THE CAR
    #BASED ON LOOKING AT DOCUMENTATION WE EITHER WILL USE TRAFFIC MANAGER OR ACTOR FUNCTIONS (THERE MAY BE ISSUES IF BOTH ARE USED)

#MAKE DESCRIPTION COMMENTS LIKE THIS FOR EACH CLASS (BUT SMALLER) FOR WHEN WORKING ON EACH OTHERS CODE 
# list all attributes, their types and a tiny description, each method should have the parameters and return values and a tiny description

"""
===========
World Class
    pub default_spawn   | first spawn point on map

create instance to initialize client connection

get_client()        | returns carla client object

get_world()         | returns carla world object

change_map(world_map: string)  | changes carla map

get_blueprints()    | returns carla blueprint library object

get_spawnpoints()   | returns list of map spawn points
===========
"""
class World:
    def __init__(self, world_map='town10HD_Opt'):
        self.__client = carla.Client('localhost', 2000)
        self.__client.set_timeout(30.0)

        cur_map = os.path.basename(self.__client.get_world().get_map().name)
        cur_map = cur_map[0].lower() + cur_map[1:]

        #CAN MAYBE ADJUST THIS TO JUST ALWAYS GET OR LOAD FROM A VALUE INSTEAD OF CHECKING?
        if world_map == cur_map:
            self.__world = self.__client.get_world()
        else:
            self.__world = self.__client.load_world(world_map)

        self.default_spawn = self.__world.get_map().get_spawn_points()[0]

    def get_client(self):
        return self.__client

    def get_world(self):
        return self.__world

    def change_map(self, world_map='town10HD_Opt'):
        return self.__world.load_world(world_map)

    def get_blueprints(self):
        return self.__world.get_blueprint_library()
    
    def get_spawnpoints(self):
        return self.__world.get_map().get_spawn_points()
    

#MAYBE ADJUST THIS CLASS TO TAKE IN A TYPE OF CAR TO BE INPUT INTO THE FILTER?
class Vehicle: 
    def __init__(self, blueprint_lib, world_map, spawn):
        vehicle_bp = random.choice(blueprint_lib.filter('vehicle.bmw.*'))
        self.__car = world_map.spawn_actor(vehicle_bp, spawn)
        self.__car.set_autopilot(True)

    def get_car(self):
        return self.__car

# sensors
"""
===========
Sensors Class

__init__ creates instance and creates list to manage all sensors

get_sensors()                    | returns list of sensors

add_sensor(sensor: Carla sensor) | adds sensor to list of sensors

destroy_sensors()                | destroys all sensors in sensor list

===========
"""
class Sensors:
    def __init__(self):
        self.__sensors = []

    def get_sensors(self):
        return self.__sensors
    
    def add_sensor(self, sensor):
        self.__sensors.append(sensor)

    def destroy_sensors(self):
        for sensor in self.__sensors:
            sensor.destroy()

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

class ObstacleSensor(Sensors):
    def __init__(self, relative_transform, parent_actor, blueprint, world):
        Sensors.__init__(self)
        self.__transform = relative_transform
        self.__parent = parent_actor
        self.__sensor = world.spawn_actor(blueprint, relative_transform, attach_to=self.__parent)
        self.__detections = []
    
    # getters
    def get_transform(self):
        return self.__transform
    def get_parent(self):
        return self.__parent
    def get_world(self):
        return self.__world
    def get_blueprint(self):
        return self.__blueprint
    def get_sensor(self):
        return self.__sensor
    def get_detections(self):
        return self.__detections
    
    
    # with event, add to list of detections
    def obstacle_detect(self, event):
        detection = (event.other_actor, event.distance)
        self.__detections.append(detection)
    
    # listen to sensor
    def listen(self):
        self.__sensor.listen(lambda event: self.obstacle_detect(event))

    
"""
===========
ObstacleSensor Class

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

class CollisionSensor(Sensors):
    def __init__(self, relative_transform, parent_actor, blueprint, world):
        Sensors.__init__(self)
        self.__transform = relative_transform
        self.__parent = parent_actor
        self.__sensor = world.spawn_actor(blueprint, relative_transform, attach_to=self.__parent)
        self.__collisions = []
    
    # getters
    def get_transform(self):
        return self.__transform
    def get_parent(self):
        return self.__parent
    def get_world(self):
        return self.__world
    def get_blueprint(self):
        return self.__blueprint
    def get_sensor(self):
        return self.__sensor
    def get_collisions(self):
        return self.__collisions
    
    
    # with event, add to list of detections
    def collision_detect(self, event):
        # other impulse is a change in momentum - indicates magnitute and direction in global coordinates
        collision = (event.actor, event.other_actor, event.normal_impulse)
        self.__collisions.append(collision)
    
    # listen to sensor
    def listen(self):
        self.__sensor.listen(lambda event: self.collision_detect(event))

"""
===========
LaneInvasionsSensor Class

__init__(self, relative_transform, parent_actor, blueprint, world) creates instance and initilizes attributes
    self.__transform = where the sensor is in relation to parent
    self.__parent = the Carla object the sensor is attached to
    self.__sensor = the Carla sensor being created
    self.__lane_invasions = list of all lane invasions by this sensor

getters for each of these attributes

lane_invasion(self, event) | adds collision (parent actor which invaded lane, line markings which were crossed) to self.__lane_invasions

listen(self) | retreives data from sensor and calls the lane_invasion method
===========
"""

class LaneInvasionSensor(Sensors):
    def __init__(self, relative_transform, parent_actor, blueprint, world):
        Sensors.__init__(self)
        self.__transform = relative_transform
        self.__parent = parent_actor
        self.__sensor = world.spawn_actor(blueprint, relative_transform, attach_to=self.__parent)
        self.__lane_invasions = []
    
    # getters
    def get_transform(self):
        return self.__transform
    def get_parent(self):
        return self.__parent
    def get_world(self):
        return self.__world
    def get_blueprint(self):
        return self.__blueprint
    def get_sensor(self):
        return self.__sensor
    def get_collisions(self):
        return self.__collisions
    
    
    # with event, add to list of detections
    def lane_invasion(self, event):
        # actor is actor that sensor is attached to and invaded another lane
        lane_invasion = (event.actor, event.crossed_lane_markings)
        self.__lane_invasions.append(lane_invasion)
    
    # listen to sensor
    def listen(self):
        self.__sensor.listen(lambda event: self.lane_invasion(event))

def main ():
    #MAYBE BREAK OUT SOME VALUES TO SET THESE MANUALLY SO WE CAN TEST DIFFERENT ENVIRONMENTS
    #initializing the world and spawns

    #CAN THIS BE DONE AS WE GO OR DO WE THINK IT IS BEST TO INITALIZE ALL VARIABLES TOGETHER?
    world = World('/Game/Carla/Maps/Town01')
    client = world.get_client()
    map = world.get_world()
    spts = world.get_spawnpoints()
    #IS SPAWN THE SAME AS DEFAULT SPAWN OR WHAT IS DEFAULT SPAWN THEN
    spawn = spts[0]
    blueprint_lib = world.get_blueprints()


    # Spectators - MOVE INTO SENSORS?
    spectator = map.get_spectator()
    spec_trans = spectator.get_transform()
    spectator.set_transform(carla.Transform(carla.Location(
        x = spawn.location.x, y = spawn.location.y, z = spawn.location.z + 60)))

    # actor list - MAYBE KEEP IN THE WORLD CLASS?
    actor_list = []

    
    vehicle = Vehicle(blueprint_lib, map, spawn)
    car = vehicle.get_car()
    actor_list.append(car)

    #MAKE A CLASS? (COULD BE CALLED END PROGRAM OR KEYBOARD CONTROL)
    while True:
        try:
            # Move the spectator behind the vehicle 
            transform = carla.Transform(car.get_transform().transform(carla.Location(x=-4,z=2.5)),car.get_transform().rotation) 
            spectator.set_transform(transform) 
            time.sleep(0.005)

        except KeyboardInterrupt:
            try: 
                #MAYBE THIS IS ITS OWN CLASS OR A FUNCTION IN WORLD
                for actor in actor_list:
                    actor.destroy()
                    print('\nCar Destroyed. Bye!')
            finally:
                break
            

main()
