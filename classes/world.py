from classes.collisionRule import CollisionRule
from classes.trafficRule import TrafficRule
from classes.pedestrianRule import PedestrianRule
from classes.parkedRule import ParkedRule
from classes.vehicle import Vehicle

import traceback
import carla
import os
import random
import time

class World:
    """
    Holds all of the variables and methods to set up and destroy the Carla world
    """
    def __init__(self, world_map='town01'):
        """
        Sets up the variables needed for the World class
        \n\tINPUT(S): world_map: [optional] world map name
        \n\tOUTPUT(S): N/A
        """
        self.__client = carla.Client('localhost', 2000)
        self.__client.set_timeout(60.0)

        cur_map = os.path.basename(self.__client.get_world().get_map().name)
        cur_map = cur_map[0].lower() + cur_map[1:]

        if world_map == cur_map:
            self.__world = self.__client.get_world()
        else:
            self.__world = self.__client.load_world(world_map)

    def get_client(self):
        """
        Getter for self.__client
        \n\tINPUT(S): N/A
        \n\tOUTPUT(S): the carla client
        """
        return self.__client

    def get_world(self):
        """
        \n\tINPUT(S): N/A
        \n\tOUTPUT(S): the carla world
        """
        return self.__world

    def get_map(self):
        """
        Getter for the map
        \n\tINPUT(S): N/A
        \n\tOUTPUT(S): the carla map
        """
        return self.__world.get_map()
    
    def get_actors(self):
        """
        Getter for the actors
        \n\tINPUT(S): N/A
        \n\tOUTPUT(S): the carla actors list
        """
        return self.__world.get_actors()

    def change_map(self, world_map='town10HD_Opt'):
        """
        Setter for the map
        \n\tINPUT(S): the world map (optional)
        \n\tOUTPUT(S): the new carla world object
        """
        return self.__world.load_world(world_map)

    def get_blueprints(self):
        """
        Getter for the blueprints
        \n\tINPUT(S): N/A
        \n\tOUTPUT(S): returns the carla blueprint libary
        """
        return self.__world.get_blueprint_library()

    def get_spawnpoints(self):
        """
        Getter for the spawnpoints
        \n\tINPUT(S): N/A
        \n\tOUTPUT(S): returns the carla spawnpoints for the map
        """
        return self.__world.get_map().get_spawn_points()

    def init_world(self):
        """
        Sets up all of the necessary parts of the world
        \n\tINPUT(S): N/A
        \n\tOUTPUT(S): returns the carla client, the world's spawnpoints, the vehicle spawn, and the carla blueprints library
        """
        return (self.__client,
            self.get_spawnpoints(),
            self.get_spawnpoints()[142],
            self.get_blueprints())

    def init_spectator(self, spawn):
        """
        Sets up the spectator for our car
        \n\tINPUT(S): spawn: the spawn transform for the vehicle
        \n\tOUTPUT(S): N/A
        """
        try:
            assert isinstance(spawn, carla.libcarla.Transform)
        except:
            raise

        spectator = self.__world.get_spectator()
        spectator.set_transform(carla.Transform(carla.Location(x = spawn.location.x, y = spawn.location.y, z = spawn.location.z + 60)))

        if type(spectator) == carla.libcarla.Actor:
            return spectator
        else:
            raise
    
    def spawn_vehicles(self):
        """
        Spawns the necessary vehicles into the Carla environment
        \n\tINPUT(S): N/A
        \n\tOUTPUT(S): N/A
        """
      # vehicle blueprints
        v_bps = self.get_blueprints().filter("vehicle.mercedes*")

        spwn_pts = self.get_spawnpoints()
        v_spwn_pts = [spwn_pts[13]]  
        
        for i in range(len(v_spwn_pts)):
            v_bp = random.choice(v_bps)
            spawn = v_spwn_pts[i]

            self.__world.spawn_actor(v_bp, spawn)
            
    def spawn_pedestrians(self): 
        """
        Spawns the necessary pedestrians into the Carla environment
        \n\tINPUT(S): N/A
        \n\tOUTPUT(S): N/A
        """
        # pedestrian blueprints
        pd_bps = self.get_blueprints().filter("walker.pedestrian.*")
        spwn_pts = self.get_spawnpoints()

        static_spawn = carla.Transform()
        static_spawn.location = spwn_pts[141].location
        self.__world.spawn_actor(random.choice(pd_bps), static_spawn)

        good_spwn_pts = [spwn_pts[95], spwn_pts[102], spwn_pts[116]]
        dest_points = [spwn_pts[203], spwn_pts[112], spwn_pts[115]]
        # spawn 10 pedestrians
        for i in range(len(good_spwn_pts)):
            pd_bp = random.choice(pd_bps)

            # remove pedestrian spawn choice to avoid collision
            spawn = carla.Transform()

            # at 10
            # choose random location, then remove that spawnpoint to avoid pedestrians from not spawning

            spawn.location = (good_spwn_pts[i]).location

            ped = self.__world.spawn_actor(pd_bp, spawn)
            control_bp = self.get_blueprints().find('controller.ai.walker')

            controller = self.__world.spawn_actor(control_bp, carla.Transform(), ped)
            controller.start()
            controller.set_max_speed(2.0)
            destination = dest_points[i]
            self.__world.tick()
            controller.go_to_location(destination.location)

    #initialize the list of actors
    def init_actors(self, spawn, blueprint_lib, spts, world, scanning_distance, dt, extra, u_nom, alpha, max_acc, max_brake, distance, standard_distance):
        """
        Initializes all actors in the Carla world
        \n\tINPUT(S): spawn: the spawn point of the vehicle,
                      blueprint_lib: the carla blueprint library,
                      spts: the spawnpoints for the maps,
                      world: the world object,
                      scanning_distance: how far ahead the vehicle looks ahead
                      DT: the tick speed,
                      extra: a multipler for CBF,
                      u_nom: a constant for CBF,
                      alpha: a constant for CBF,
                      max_acc: the maximum acceleration,
                      max_brake: the maximum brake value,
                      distance: the base distance for vehicle
                      standard_distance: the standard following distance for the vehicle

        \n\tOUTPUT(S):
        """        
        try:
            assert type(spawn) == carla.libcarla.Transform
            assert type(blueprint_lib) == carla.libcarla.BlueprintLibrary
            assert type(spts) == list
            assert type(world) == World
        except:
            raise
        
        transforms = [carla.Transform(carla.Location(x=2.8, z=0.7)), carla.Transform(carla.Location(x=4.8, z=0.7)), carla.Transform(carla.Location(x=6.8, z=0.7))]
        blueprints = [blueprint_lib.find('sensor.other.obstacle'), blueprint_lib.find('sensor.other.collision'), blueprint_lib.find('sensor.other.lane_invasion')]
        blueprints[0].set_attribute('distance', scanning_distance)

        vehicle = Vehicle(blueprint_lib, world.get_world(), spawn, spts[5], transforms, blueprints, world, dt, extra, u_nom, alpha, max_acc, max_brake, distance, standard_distance)
        car = vehicle.get_car()

        # vehicle.set_sensors(transforms, car, blueprints, world, blueprint_lib)
        rules = [CollisionRule(vehicle.get_sensors(), vehicle), PedestrianRule(vehicle.get_sensors(),vehicle), TrafficRule(vehicle.get_sensors(), vehicle), ParkedRule(vehicle.get_sensors(), vehicle)]
        vehicle.set_rules(rules)

        world.spawn_pedestrians()
        world.spawn_vehicles()

        if type(vehicle) == Vehicle and type(car) == carla.libcarla.Vehicle:
            return (vehicle, car)
        else:
            raise

    def clear_world(self, client):
        """
        Reloads the world so that no errors occur
        \n\tINPUT(S): client: the carla client
        \n\tOUTPUT(S): N/A
        """              
        try:
            assert type(client) == carla.libcarla.Client
        except:
            print(traceback.format_exc())

        client.reload_world()
        print("World cleared :)\n")


    #repeating logic performed in the main function
    def main_loop(self, spectator, car, vehicle, DT):
        """
        Tne function which holds the functions to be repeated each cycle
        \n\tINPUT(S): spectator: the carla spectator,
                      car: the carla vehicle object,
                      vehicle: the Vehicle object,
                      DT: tick speed
        \n\tOUTPUT(S): N/A
        """      
        try:
            assert type(car) == carla.libcarla.Vehicle
            assert type(vehicle) == Vehicle
        except:
            raise

        # Move the spectator behind the vehicle
        car_rotation = car.get_transform().rotation
        transform = carla.Transform(car.get_transform().transform(carla.Location(x=-3,z=5)), carla.Rotation(car_rotation.pitch - 30.0, car_rotation.yaw, car_rotation.roll))
        spectator.set_transform(transform)
        time.sleep(DT)
        try:
            vehicle.mape_drive()
        except:
            raise