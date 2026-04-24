from classes.world import World
from classes.collisionException import CollisionErr
from classes.routeDone import RouteDone
import traceback

# STATIC VARIABLES
dt = 0.005
extra = 3
standard_distance = 8
u_nom = 1
alpha = 1.5
max_acc = 3
max_brake = 6
distance = 16
scanning_distance = '35.0'

def main():
    """
    Controls the main function of the carla program.
    Holds the functions that are run when the program begins.
    Holds all of the exception handling from throughout the program.
    """ 
    world = World()
    client, spts, spawn, blueprint_lib = world.init_world()

    try:
        spectator = world.init_spectator(spawn)
        vehicle, car = world.init_actors(spawn, blueprint_lib, spts, world, scanning_distance, dt, standard_distance, u_nom, alpha, max_acc, max_brake, distance, extra)
        

        while True:
            world.main_loop(spectator, car, vehicle, dt)
            world.get_world().tick()

    except CollisionErr:
        print("Collision")
    except RouteDone:
        print("Route done!")
    except Exception:
        print(traceback.format_exc())
    except KeyboardInterrupt:
        print(" Keyboard Interrupt")
    finally:
        world.clear_world(client)

if __name__ == "__main__":
    main()
