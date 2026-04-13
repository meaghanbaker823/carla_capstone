from classes.world import World
from classes.collision_exception import CollisionErr
from classes.route_done import RouteDone
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
    world = World()
    client, spts, spawn, blueprint_lib = world.init_world()
    global control_flag
    control_flag = True

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

"""
================================================================
# Parameters
DT = 0.05
D0 = 8.0     # nominal safe distance (m)
D_EXTRA = 6.0
ALPHA = 1.5
MAX_ACCEL = 3.0        # m/s^2
MAX_BRAKE_DECEL = 6.0  # m/s^2
 
# Nominal acceleration (throttle) for cruise
u_nom = 1.0
 
# Compute safety function
d = distance_to_crosswalk(vehicle)
d_min = D0 + occlusion_risk(vehicle) * D_EXTRA
h = d - d_min
 
v = get_speed(vehicle)
 
# Compute maximum allowed acceleration for CBF
u_cbf = (ALPHA * h - v) / DT
 
# Clamp with nominal and actuator limits
u = min(u_nom, u_cbf, MAX_ACCEL)
u = max(u, -MAX_BRAKE_DECEL)
 
# Map to CARLA controls
if u >= 0:
    throttle = u / MAX_ACCEL
    brake = 0.0
else:
    throttle = 0.0
    brake = min(-u / MAX_BRAKE_DECEL, 1.0)
 
vehicle.apply_control(carla.VehicleControl(throttle=throttle, brake=brake))
"""