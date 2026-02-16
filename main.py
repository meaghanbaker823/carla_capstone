# imports
import carla
import time
import traceback

from classes.vehicle import Vehicle
from classes.navigation import MainNavigator
from classes.world import World
from classes.collisionRule import CollisionRule
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

    # nav = MainNavigator(spawn, spts[5], world.get_world())
    
    transforms = [carla.Transform(carla.Location(x=2.8, z=0.7)), carla.Transform(carla.Location(x=4.8, z=0.7)), carla.Transform(carla.Location(x=6.8, z=0.7))]
    blueprints = [blueprint_lib.find('sensor.other.obstacle'), blueprint_lib.find('sensor.other.collision'), blueprint_lib.find('sensor.other.lane_invasion')]
    blueprints[0].set_attribute('distance', '20.0')
    vehicle = Vehicle(blueprint_lib, world.get_world(), spawn, spts[5], transforms, blueprints, world)
    car = vehicle.get_car()
    vehicle.set_sensors(transforms, car, blueprints, world, blueprint_lib)
    rules = [CollisionRule(vehicle.get_sensors(), vehicle), TrafficRule(vehicle.get_sensors(), vehicle)]
    vehicle.set_rules(rules)
    speed_threshold = 3
    checks = [SpeedOverCheck(speed_threshold), SpeedPerfectCheck(speed_threshold), SpeedUnderCheck(speed_threshold)]
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
    # if(vehicle.control_loop() == False):
    if(not vehicle.mape_drive(30)):
        print(traceback.format_exc())
        return False

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
            if(main_loop(spectator, car, vehicle)):
                raise Exception()

    except Exception:
        print(traceback.format_exc())
    except KeyboardInterrupt:
        print(" Keyboard Interrupt")
    finally:
        clear_world(client)

if __name__ == "__main__":
    main()
