import carla
import os

"""
===========
World Class

create instance to initialize client connection

get_client()        | returns carla client object

get_world()         | returns carla world object

change_map(string)  | changes carla map

get_blueprints()    | returns carla blueprint library object
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

    def get_client(self):
        return self.__client

    def get_world(self):
        return self.__world

    def change_map(self, world_map='town10HD_Opt'):
        return self.__world.load_world(world_map)

    def get_blueprints(self):
        return self.__world.get_blueprint_library()
