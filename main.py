from classes.vehicle import Vehicle
from classes.world import World
from classes.collisionRule import CollisionRule
from classes.trafficRule import TrafficRule
from classes.speedOverCheck import SpeedOverCheck
from classes.speedPerfectCheck import SpeedPerfectCheck
from classes.speedUnderCheck import SpeedUnderCheck  
from classes.collision_exception import CollisionErr
from classes.route_done import RouteDone
import carla
import time
import traceback

"""
- work on checking exceptions (make them work how they should)
- fix control flag so it works (or change it)
- if MAPE is working, clean up non-MAPE
- if extra time, clean up main
- make sure keyboard interrupt works throughout program
- check waypoints are advancing correctly (why is car crashing?) 
- make car stop before collision (either make car stop and/or make program fully stop when collision happens)
"""

#initialize the list of actors
def init_actors(spawn, blueprint_lib, spts, world):
    try:
        assert type(spawn) == carla.libcarla.Transform
        assert type(blueprint_lib) == carla.libcarla.BlueprintLibrary
        assert type(spts) == list
        assert type(world) == World
    except:
        raise
    
    transforms = [carla.Transform(carla.Location(x=2.8, z=0.7)), carla.Transform(carla.Location(x=4.8, z=0.7)), carla.Transform(carla.Location(x=6.8, z=0.7))]
    blueprints = [blueprint_lib.find('sensor.other.obstacle'), blueprint_lib.find('sensor.other.collision'), blueprint_lib.find('sensor.other.lane_invasion')]
    blueprints[0].set_attribute('distance', '20.0')

    vehicle = Vehicle(blueprint_lib, world.get_world(), spawn, spts[5], transforms, blueprints, world)
    car = vehicle.get_car()

    # vehicle.set_sensors(transforms, car, blueprints, world, blueprint_lib)
    rules = [CollisionRule(vehicle.get_sensors(), vehicle), TrafficRule(vehicle.get_sensors(), vehicle)]
    vehicle.set_rules(rules)

    speed_threshold = 3
    checks = [SpeedOverCheck(speed_threshold), SpeedPerfectCheck(speed_threshold), SpeedUnderCheck(speed_threshold)]
    vehicle.set_checks(checks)

    world.spawn_pedestrians()

    if type(vehicle) == Vehicle and type(car) == carla.libcarla.Vehicle:
        return (vehicle, car)
    else:
        raise

#repeating logic performed in the main function
def main_loop(spectator, car, vehicle):
    try:
        assert type(car) == carla.libcarla.Vehicle
        assert type(vehicle) == Vehicle
    except:
        raise

    # Move the spectator behind the vehicle
    transform = carla.Transform(car.get_transform().transform(carla.Location(x=-4,z=2.5)),car.get_transform().rotation)
    spectator.set_transform(transform)
    time.sleep(0.005)
    try:
        vehicle.mape_drive(30)
    except:
        raise

def clear_world(client):
    try:
        assert type(client) == carla.libcarla.Client
    except:
        print(traceback.format_exc())

    client.reload_world()
    print("World cleared :)\n")

def main():   
    world = World()
    client, spts, spawn, blueprint_lib = world.init_world()
    global control_flag
    control_flag = True

    try:
        spectator = world.init_spectator(spawn)

        vehicle, car = init_actors(spawn, blueprint_lib, spts, world)

        while True:
            main_loop(spectator, car, vehicle)

    except CollisionErr:
        print("Collision")
    except RouteDone:
        print("Route done!")
    except Exception:
        print(traceback.format_exc())
    except KeyboardInterrupt:
        print(" Keyboard Interrupt")
    finally:
        clear_world(client)

if __name__ == "__main__":
    main()
