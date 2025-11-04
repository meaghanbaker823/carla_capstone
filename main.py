# imports
import carla
from carla import ColorConverter as cc
import os
import random
import time
import numpy as np
import weakref
import cv2
import math


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
    
# there are intersection entrance actors, stop sign actors, traffic light actors
#TODO: command list for Vehicle

class Vehicle: 
    def __init__(self, blueprint_lib, world_map, spawn):
        vehicle_bp = random.choice(blueprint_lib.filter('vehicle.bmw.*'))
        self.__car = world_map.spawn_actor(vehicle_bp, spawn)
        self.__actors = world_map.get_actors()
        self.__sensors = Sensors()
  
    def get_car(self):
        return self.__car
    
    def get_sensors(self):
        return self.__sensors
    

    def set_sensors(self, transform, actor, blueprint, world):
        self.__sensors.add_sensor(ObstacleSensor(transform[0], actor, blueprint[0], world))
        self.__sensors.add_sensor(CollisionSensor(transform[1], actor, blueprint[1], world))
        self.__sensors.add_sensor(LaneInvasionSensor(transform[2], actor, blueprint[2], world))
    

        for sensor in self.__sensors.get_sensors():
            sensor.listen()

    def control_loop(self):
        if((self.__sensors.get_sensors()[0].get_other_actors() == []) or (self.__sensors.get_sensors()[2].get_lane_markings() == [])):
            if(len(self.__sensors.get_sensors()[0].get_other_actors()) > 0):
                self.avoid_obstacles()

            elif(len(self.__sensors.get_sensors()[1].get_collisions()) > 0):
                self.stop_car()
                return False

            elif(len(self.__sensors.get_sensors()[2].get_lane_markings()) > 0):
                self.fix_lane()
        else:
            self.drive()


    def maintain_speed(self, s):
        PREFERRED_SPEED = 30        # targeted speed in kph
        SPEED_THRESHOLD = 2         # how many kph we can comfortably be under the target
        if s >= PREFERRED_SPEED:
            return 0
        elif s < PREFERRED_SPEED - SPEED_THRESHOLD:
            return 0.8      # essentially 80% of gas
        else:   
            return 0.4      
        
    def drive(self):   
        v = self.get_car().get_velocity()                                   # velocity is a 3d vector in m/s
        speed = round(3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2),0)          # speed in kilometers/hr 
        steering_angle = 0  # you can change this but once it gets near an obsticle it will stop in a straight line

        estimated_throttle = self.maintain_speed(speed)
        self.get_car().apply_control(carla.VehicleControl(throttle=estimated_throttle,steer=steering_angle))
    
    def decelerate(self):
        self.get_car().apply_control(carla.VehicleControl(throttle=0, steer=1.0, brake=0.7))  # cuts throttle and applies 70% braking power

   
    def avoid_obstacles(self):
        # self.decelerate()
        print("Decelerating")
        self.get_car().apply_control(carla.VehicleControl(throttle=0.4,steer=-1.0))

        self.__sensors.get_sensors()[0].delete_old_detection()
    
    def stop_car(self):
        self.get_car().apply_control(carla.VehicleControl(throttle=0,steer=0,brake=1))
        print("Stopping Car")
        # print(Vehicle.get_car(self).get_velocity())
    
    def fix_lane(self):
        return True


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
        print(f"{len(self.__sensors)}heloooooooo")
        for sensor in self.__sensors:
            print("Sensor destroyed.")
            print(sensor)
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
        self.__other_actors = []
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
    def get_other_actors(self):
        return self.__other_actors
    
    def delete_old_detection(self):
        self.__detections.pop(0)
        self.__other_actors.pop(0)
          
    # with event, add to list of detections
    def obstacle_detect(self, event):
        if event.other_actor not in self.__other_actors:
            detection = (event.other_actor, event.distance)
            self.__detections.append(detection)
            self.__other_actors.append(event.other_actor)

    
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
        self.__lane_markings = []
    
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
    def get_lane_invasions(self):        
        return self.__lane_invasions
    def get_lane_markings(self):
        return self.__lane_markings
    

    def delete_old_detection(self):
        self.__lane_invasions.pop(0)
        self.__lane_markings.pop(0)
    
    # with event, add to list of detections
    def lane_invasion(self, event):
        # actor is actor that sensor is attached to and invaded another lane
        #THIS LINE HAS A RUNTIME ERROR (TRYING TO OPERATE ON A DESTROYED ACTOR)
        lane_invasion = (event.actor, event.crossed_lane_markings)
        self.__lane_invasions.append(lane_invasion)
        self.__lane_markings.append(event.crossed_lane_markings)
    
    # listen to sensor
    def listen(self):
        self.__sensor.listen(lambda event: self.lane_invasion(event))



def main ():
    #initializing the world and spawns

    world = World('/Game/Carla/Maps/Town01')
    client = world.get_client()
    map = world.get_world()
    spts = world.get_spawnpoints()
    spawn = spts[0]
    blueprint_lib = world.get_blueprints()


    # Spectators
    spectator = map.get_spectator()
    spec_trans = spectator.get_transform()
    spectator.set_transform(carla.Transform(carla.Location(
        x = spawn.location.x, y = spawn.location.y, z = spawn.location.z + 60)))

    # actor list
    actor_list = []
    
    vehicle = Vehicle(blueprint_lib, map, spawn)
    car = vehicle.get_car()
    actor_list.append(car)
    transforms = [carla.Transform(carla.Location(x=2.8, z=0.7)), carla.Transform(carla.Location(x=4.8, z=0.7)), carla.Transform(carla.Location(x=6.8, z=0.7))]
    blueprints = [blueprint_lib.find('sensor.other.obstacle'), blueprint_lib.find('sensor.other.collision'), blueprint_lib.find('sensor.other.lane_invasion')]
    blueprints[0].set_attribute('distance', '15.0')
    vehicle.set_sensors(transforms, car, blueprints, map)
    actor_list.append(vehicle.get_sensors().get_sensors()[0])
    actor_list.append(vehicle.get_sensors().get_sensors()[1])
    actor_list.append(vehicle.get_sensors().get_sensors()[2])


    while True:
        try:
            # Move the spectator behind the vehicle 
            transform = carla.Transform(car.get_transform().transform(carla.Location(x=-4,z=2.5)),car.get_transform().rotation) 
            spectator.set_transform(transform) 
            time.sleep(0.005)
            vehicle.drive()
            if(vehicle.control_loop() == False):
                raise KeyboardInterrupt()

        except KeyboardInterrupt:
            try: 
                client.reload_world()
                print("World cleared :)\n")
  
            finally:
                break
            

main()
