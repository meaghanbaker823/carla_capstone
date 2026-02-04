# imports
import carla
import os
import random
import time
import numpy as np
import cv2

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
    
class Vehicle: 
    def __init__(self, blueprint_lib, world_map, spawn):
        vehicle_bp = random.choice(blueprint_lib.filter('vehicle.bmw.*'))
        self.__car = world_map.spawn_actor(vehicle_bp, spawn)
        self.__car.set_autopilot(True)
    def get_car(self):
        return self.__car

# TODO: implment abby's sensors

class Sensors:
    def __init__(self):
        pass

class ObstacleSensor(Sensors):
    def __init__(self):
        pass

class CollisionSensors(Sensors):
    def __init__(self):
        pass

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

    while True:
        try:
            # Move the spectator behind the vehicle 
            transform = carla.Transform(car.get_transform().transform(carla.Location(x=-4,z=2.5)),car.get_transform().rotation) 
            spectator.set_transform(transform) 
            time.sleep(0.005)

        except KeyboardInterrupt:
            try: 
                for actor in actor_list:
                    actor.destroy()
                    print('\nCar Destroyed. Bye!')
            finally:
                break
            

main()
