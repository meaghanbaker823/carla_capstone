from classes.world import World
from classes.collision_exception import CollisionErr
from classes.route_done import RouteDone
import traceback

# STATIC VARIABLES
dt = 0.005
extra = 5
standard_distance = 8
u_nom = 40
alpha = 0.01
max_acc = 70
distance = 20
scanning_distance = '20.0'

def main():   
    world = World()
    client, spts, spawn, blueprint_lib = world.init_world()
    global control_flag
    control_flag = True

    try:
        spectator = world.init_spectator(spawn)
        vehicle, car = world.init_actors(spawn, blueprint_lib, spts, world, scanning_distance, dt, extra, u_nom, alpha, max_acc, distance, standard_distance)

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
