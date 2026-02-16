import carla
import os
import traceback
import random

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
    def __init__(self, world_map='town01'):
        self.__client = carla.Client('localhost', 2000)
        self.__client.set_timeout(60.0)

        cur_map = os.path.basename(self.__client.get_world().get_map().name)
        cur_map = cur_map[0].lower() + cur_map[1:]

        if world_map == cur_map:
            self.__world = self.__client.get_world()
        else:
            self.__world = self.__client.load_world(world_map)

    def get_client(self):
        return self.__client

    def get_world(self):
        return self.__world

    def get_map(self):
        return self.__world.get_map()
    
    def get_actors(self):
        return self.__world.get_actors()

    def change_map(self, world_map='town10HD_Opt'):
        return self.__world.load_world(world_map)

    def get_blueprints(self):
        return self.__world.get_blueprint_library()

    def get_spawnpoints(self):
        return self.__world.get_map().get_spawn_points()

    def init_world(self):
        return (self.__client,
            self.get_map(),
            self.get_spawnpoints(),
            self.get_spawnpoints()[8],
            self.get_blueprints())

    def init_spectator(self, spawn):
        try:
            assert type(spawn) == carla.libcarla.Transform
        except:
            print(traceback.format_exc())

        try:
            assert type(spawn) == carla.libcarla.Transform
        except:
            print(traceback.format_exc())

        spectator = self.__world.get_spectator()
        spectator.set_transform(carla.Transform(carla.Location(
            x = spawn.location.x, y = spawn.location.y, z = spawn.location.z + 60)))

        if type(spectator) == carla.libcarla.Actor:
            return spectator
        else:
            print(traceback.format_exc)
            return False
        
    def spawn_pedestrians(self):
        # pedestrian blueprints
        pd_bps = self.get_blueprints().filter("walker.pedestrian.*")
        spwn_pts = self.get_spawnpoints()


        # spawn 10 pedestrians
        for i in range(10):
            pd_bp = random.choice(pd_bps)

            # remove pedestrian spawn choice to avoid collision
            
            spawn = carla.Transform()
            # at 10
            # choose random location, then remove that spawnpoint to avoid pedestrians from not spawning
            spawn.location = (spwn_pts[10]).location
            

            ped = self.__world.spawn_actor(pd_bp, spawn)
            control_bp = self.get_blueprints().find('controller.ai.walker')
            controller = self.__world.spawn_actor(control_bp, carla.Transform(), ped)
            controller.start()
            controller.go_to_location(self.__world.get_random_location_from_navigation())