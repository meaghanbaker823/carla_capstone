# imports
import carla
import time
import traceback

from classes.vehicle import Vehicle
from classes.navigation import Navigation
from classes.trafficLight import TrafficLight
from classes.world import World
from classes.collisionRule import CollisionRule
from classes.obstacleRule import ObstacleRule
from classes.trafficRule import TrafficRule
from classes.speedOverCheck import SpeedOverCheck
from classes.speedPerfectCheck import SpeedPerfectCheck
from classes.speedUnderCheck import SpeedUnderCheck       

#initialize the list of actors
def init_actors(spawn, blueprint_lib, spts, world):
    try:
        assert type(spawn) == carla.libcarla.Transform
        assert type(blueprint_lib) == carla.libcarla.BlueprintLibrary
        assert type(spts) == list
        assert type(world) == World
    except:
        print(traceback.format_exc())

    nav = Navigation(spawn, spts[6], world.get_world())
    vehicle = Vehicle(blueprint_lib, world.get_world(), spawn, nav)
    car = vehicle.get_car()
    transforms = [carla.Transform(carla.Location(x=2.8, z=0.7)), carla.Transform(carla.Location(x=4.8, z=0.7)), carla.Transform(carla.Location(x=6.8, z=0.7))]
    blueprints = [blueprint_lib.find('sensor.other.obstacle'), blueprint_lib.find('sensor.other.collision'), blueprint_lib.find('sensor.other.lane_invasion')]
    blueprints[0].set_attribute('distance', '20.0')
    vehicle.set_sensors(transforms, car, blueprints, world, blueprint_lib)
    traffic_lights = TrafficLight(world.get_world().get_actors())
    rules = [CollisionRule(vehicle.get_sensors(), vehicle), ObstacleRule(vehicle.get_sensors(), vehicle), TrafficRule(vehicle.get_sensors(), vehicle)]
    vehicle.set_rules(rules)
    checks = [SpeedOverCheck(3), SpeedPerfectCheck(3), SpeedUnderCheck(3)]
    vehicle.set_checks(checks)
    world.spawn_pedestrians()

    if type(vehicle) == Vehicle and type(car) == carla.libcarla.Vehicle:
        return (vehicle, car)
    else:
        print(traceback.format_exc())
        return False

#repeating logic performed in the main function
def main_loop(spectator, car, vehicle):
    try:
        assert type(car) == carla.libcarla.Vehicle
        assert type(vehicle) == Vehicle
    except:
        print(traceback.format_exc())

    # Move the spectator behind the vehicle
    transform = carla.Transform(car.get_transform().transform(carla.Location(x=-4,z=2.5)),car.get_transform().rotation)
    spectator.set_transform(transform)
    time.sleep(0.005)
    if(vehicle.control_loop() == False):
        raise KeyboardInterrupt()

def clear_world(client):
    try:
        assert type(client) == carla.libcarla.Client
    except:
        print(traceback.format_exc())

    client.reload_world()
    print("World cleared :)\n")

def main():   
    world = World()
    client, map, spts, spawn, blueprint_lib = world.init_world()

    try:
        spectator = world.init_spectator(spawn)

        vehicle, car = init_actors(spawn, blueprint_lib, spts, world)

        while True:
            main_loop(spectator, car, vehicle)

    except Exception:
        print(traceback.format_exc())
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
    finally:
        clear_world(client)

if __name__ == "__main__":
    main()
