import arin_segment
import random
import carla

def main():
    try: 
        #initializes world
        world = arin_segment.World()

        #sets random values (blueprint and spawn point) and creates car
        bp = random.choice(world.get_blueprints().filter('vehicle'))
        transform = random.choice(world.get_spawn_points())
        vehicle = world.spawn_actor(bp, transform)

        #turn on vehicles lights
        vehicle.set_autopilot(True)
    
    finally:
        carla.command.DestroyActor(vehicle)


main()
