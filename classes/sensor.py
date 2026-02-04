class Sensor:
    def __init__(self, relative_transform, parent_actor, blueprint, world):
        self.__transform = relative_transform
        self.__parent = parent_actor
        self.__sensor = world.get_world().spawn_actor(blueprint, relative_transform, attach_to=self.__parent)

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
    
    def listen():
        pass