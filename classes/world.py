import carla
import os
import random

"""
===========
World Class()
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
            self.get_spawnpoints(),
            self.get_spawnpoints()[142],
            self.get_blueprints())

    def init_spectator(self, spawn):
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
      # vehicle blueprints
        v_bps = self.get_blueprints().filter("vehicle.*")

        spwn_pts = self.get_spawnpoints()
        v_spwn_pts = [spwn_pts[13], spwn_pts[117], spwn_pts[114], spwn_pts[141]]  
        
        for i in range(len(v_spwn_pts)):
            v_bp = random.choice(v_bps)
            spawn = v_spwn_pts[i]

            self.__world.spawn_actor(v_bp, spawn)
            



    def spawn_pedestrians(self): 
        # pedestrian blueprints
        pd_bps = self.get_blueprints().filter("walker.pedestrian.*")
        spwn_pts = self.get_spawnpoints()
        good_spwn_pts = [spwn_pts[95], spwn_pts[102], spwn_pts[202], spwn_pts[116]]
        # good_spwn_pts[0].location.z -= 1
        dest_points = [spwn_pts[203], spwn_pts[112], spwn_pts[7], spwn_pts[115]]

        # spawn 10 pedestrians
        for i in range(len(good_spwn_pts)):
            pd_bp = random.choice(pd_bps)
            print(i)

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
