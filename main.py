# imports
import carla
from carla import ColorConverter as cc
import os
import random
import time
import numpy as np
import weakref
import cv2
import math
import traceback
from global_route_planner import GlobalRoutePlanner


"""
===========
World Class
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
            self.get_map(),
            self.get_spawnpoints(),
            self.get_spawnpoints()[0],
            self.get_blueprints())

    def init_spectator(self, spawn):
        spectator = self.__world.get_spectator()
        spectator.set_transform(carla.Transform(carla.Location(
            x = spawn.location.x, y = spawn.location.y, z = spawn.location.z + 60)))

        return spectator
    
"""
===========
Vehicle Class

__init__ creates instance and creates list to manage a vehicle

control_loop()                  | sets color by processing the state of the light

set_sensors()                   | moves the vehicle based on the color

is_car_moving()                 | detects if light is applicable

maintain_speed()                | adjusts throttle based on current speed to adjust to current speed

drive()                         | drives the car based on navigator

decelerate()                    | slows down car with brake

stop_car()                      | stops car by applying full brake

fix_lane()                      | adjusts car into the center of the lane

avoid_obstacles()               | will avoid obstacles detected
===========
"""
class Vehicle: 
    def __init__(self, blueprint_lib, world_map, spawn, navigator):
        self.__car = world_map.spawn_actor(random.choice(blueprint_lib.filter('vehicle.bmw.*')), spawn)
        self.__actors = world_map.get_actors()
        self.__sensors = []
        self.__navigator = navigator
        self.__speed_limit = 30
    
    def get_sensors(self):
        return self.__sensors
    
    def add_sensor(self, sensor):
        self.__sensors.append(sensor)

    def get_navigator(self):
        return self.__navigator
  
    def get_car(self):
        return self.__car
    
    def get_sensors(self):
        return self.__sensors
    
    def get_speed_limit(self):
        return self.__speed_limit
    
    def get_actors(self):
        return self.__actors
    
    def set_speed_limit(self ,speed):
        self.__speed_limit = speed
    
    def set_sensors(self, transform, actor, blueprint, world, blueprint_lib):
        self.add_sensor(ObstacleSensor(transform[0], actor, blueprint[0], world, blueprint_lib))
        self.add_sensor(CollisionSensor(transform[1], actor, blueprint[1], world, blueprint_lib))
    
        for sensor in self.__sensors:
            sensor.listen()

    def control_loop(self):
        car_changed = False

        if(self.__sensors[1].get_collisions() != []):
            self.stop_car()
            return False
        
        if(self.__sensors[0].get_other_actors() != []):
            car_changed = True
            self.avoid_obstacles()

        traffic_lights = TrafficLight(self.get_actors())
        if(traffic_lights.process_color(self.get_car()) != ""):
            car_changed = True
            if(traffic_lights.get_response() == "stop"):
                self.stop_car()
            else:
                self.drive()

        if(not car_changed):
            if(not self.drive()):
                return False
    
    def is_car_moving(self):
        return self.get_car().get_velocity()
 

    def maintain_speed(self, current):
        try:
            assert current >= 0
        except Exception as e:
            print(traceback.format_exc())
        SPEED_THRESHOLD = 2         # how many kph we can comfortably be under the target
        if current >= self.get_speed_limit():
            return 0
        elif current < self.get_speed_limit() - SPEED_THRESHOLD:
            return 0.8      # essentially 80% of gas
        else:   
            return 0.4
        
    def drive(self):
        try:
            if self.__navigator.get_cur_waypoint_num() >= (len(self.__navigator.get_route()) - 6):
                return False
            self.__navigator.advance_waypoint(self.__car)
            v = self.get_car().get_velocity()                                   # velocity is a 3d vector in m/s
            speed = round(3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2),0)          # speed in kilometers/hr 
            
            waypt_nm, steering_angle = self.__navigator.get_proper_angle(self.__car, self.__navigator.get_cur_waypoint_num(), self.__navigator.get_route()) 
            
            self.__navigator.set_cur_waypoint(waypt_nm)
            steering_angle /= 75
            
            estimated_throttle = self.maintain_speed(speed)
            if (estimated_throttle == 0):
                brake_force = 1.0
            else:
                brake_force = 0
            self.get_car().apply_control(carla.VehicleControl(throttle=estimated_throttle,steer=steering_angle, brake=brake_force))
            return True
        
        except Exception as e:
            print(traceback.format_exc())
            return False

    def decelerate(self):
        self.get_car().apply_control(carla.VehicleControl(throttle=0, brake=0.7))

    def avoid_obstacles(self):
        if(self.__sensors[0].get_other_actors()[0].type_id[:-2] == "traffic.speed_limit."):
            self.set_speed_limit(30)
        self.__sensors[0].delete_old_detection()
 
    def stop_car(self):
        if(self.is_car_moving):
            self.get_car().apply_control(carla.VehicleControl(throttle=0,brake=1.0))

    def fix_lane(self):
        return True


"""
===========
TrafficLight Class

__init__ creates instance and creates list to manage traffic lights

process_color()                  | sets color by processing the state of the light

react_to_color()                 | moves the vehicle based on the color

is_light_close()                 | detects if light is applicable

===========
"""
class TrafficLight():
    def __init__(self, actors):
        self.__lights = []
        self.__color = "unknown"
        self.__response = ""
        self.set_lights(actors)

    def get_color(self):
        return self.__color
    def get_lights(self):
        return self.__lights
    def get_response(self):
        return self.__response
    
    def set_lights(self, actors):
        self.__lights = actors.filter('traffic.traffic_light*')
    def set_response(self, response):
        self.__response = response
    def set_color(self, color):
        self.__color = color

    def process_color(self, car):
        try:
            assert type(car) == carla.libcarla.Vehicle
        except Exception as e:
            print(traceback.format_exc())
        for light in self.get_lights():
            if(self.is_light_close(car, light, 10, 40)):
                old_color = self.get_color()
                self.set_color(light.get_state().name)

                if(old_color != self.get_color()):
                    # print("Traffic light is ", self.get_color())
                    self.set_response(self.react_to_color())
                    break
            else:
                self.set_response("")
  
        return self.get_response()

    def react_to_color(self):
        action = ""
        if(self.get_color() == "Green"):
            action = "drive"
        else:
            action = "stop"
        
        try:
            assert type(action) == str
        except Exception as e:
            print(traceback.format_exc())

        return action
    
    def is_light_close(self, car_check, light_check, target_distance, target_angle):
        try:
            assert type(car_check) == carla.Vehicle
            assert type(light_check) == carla.TrafficLight
            assert target_distance >= 0
            assert -180 <= target_angle <= 180
        except Exception as e:
            print(traceback.format_exc())
        
        car = car_check.get_transform().get_forward_vector()
        light = light_check.get_location() - car_check.get_location()
    
        dot_product = car.x * light.x + car.y * light.y + car.z * light.z
        magnitude_car = math.sqrt(car.x**2 + car.y**2 + car.z**2)
        magnitude_light = math.sqrt(light.x**2 + light.y**2 + light.z**2)

        angle_deg = abs(math.degrees(math.acos(max(-1.0, min(1.0, (dot_product / (magnitude_car * magnitude_light)))))))
        distance = car_check.get_location().distance(light_check.get_location())

        return ((angle_deg < target_angle) and (distance < target_distance))



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

"""
===========
ObstacleSensor Class

__init__(self, relative_transform, parent_actor, blueprint, world) creates instance and initilizes attributes
    self.__transform = where the sensor is in relation to parent
    self.__parent = the Carla object the sensor is attached to
    self.__sensor = the Carla sensor being created
    self.__detections = list of all detections by this sensor

getters for each of these attributes

obstacle_detect(self, event) | adds detection (obstactle detected, distance to that obstacle) to self.__detections

listen(self) | retreives data from sensor and calls the obstacle dection method
===========
"""

class ObstacleSensor(Sensor):
    def __init__(self, relative_transform, parent_actor, blueprint, world, blueprint_lib):
         # pre-condition
        try:
            assert type(relative_transform) == carla.libcarla.Transform # relative transform must be a transform object
            assert str(parent_actor) in [str(actor) for actor in world.get_actors()] # parent actor must be spawned in the world
            assert str(blueprint) == str(blueprint_lib.find('sensor.other.obstacle')) # blueprint must be obstacle detector
        except Exception:
            print(traceback.format_exc())
        super().__init__(relative_transform, parent_actor, blueprint, world)
        self.__other_actors = []
        self.__detections = []
    
    # getters
    def get_detections(self):
        return self.__detections
    def get_other_actors(self):
        return self.__other_actors
    
    def delete_old_detection(self):
        detections_len = len(self.__detections)
        oactorer_len = len(self.__other_actors)

        self.__detections.pop(0)
        self.__other_actors.pop(0)

         # post condition
        try:
            assert len(self.__detections) == detections_len - 1
            assert len(self.__other_actors) == oactorer_len - 1
        except Exception as e:
            print(traceback.format_exc())
          
    # with event, add to list of detections
    def obstacle_detect(self, event):
        try:
            assert type(event) == carla.libcarla.ObstacleDetectionEvent
        except Exception as e:
            print(traceback.format_exc())

        detections_len = len(self.__detections)

        if event.other_actor not in self.__other_actors:
            detection = (event.other_actor, event.distance)
            self.__detections.append(detection)
            self.__other_actors.append(event.other_actor)

             # post condition
            try:
                assert len(self.__detections) == detections_len + 1
            except Exception as e:
                print(traceback.format_exc())

    
    # listen to sensor
    def listen(self):
        self.get_sensor().listen(lambda event: self.obstacle_detect(event))

    
"""
===========
ObstacleSensor Class

__init__(self, relative_transform, parent_actor, blueprint, world) creates instance and initilizes attributes
    self.__transform = where the sensor is in relation to parent
    self.__parent = the Carla object the sensor is attached to
    self.__sensor = the Carla sensor being created
    self.__collisions = list of all collisions by this sensor

getters for each of these attributes

collision_detect(self, event) | adds collision (parent actor, other actor in collision, intensity of impact) to self.__collisions

listen(self) | retreives data from sensor and calls the collision_detect method
===========
"""

class CollisionSensor(Sensor):
    def __init__(self, relative_transform, parent_actor, blueprint, world, blueprint_lib):
         # pre-condition
        try:
            assert type(relative_transform) == carla.libcarla.Transform # relative transform must be a transform object
            assert str(parent_actor) in [str(actor) for actor in world.get_actors()] # parent actor must be spawned in the world
            assert str(blueprint) == str(blueprint_lib.find('sensor.other.collision')) # blueprint must be obstacle detector
        except Exception:
            print(traceback.format_exc())

        super().__init__(relative_transform, parent_actor, blueprint, world)
        self.__collisions = []
        
    
    # getters
    def get_collisions(self):
        return self.__collisions
    
    # with event, add to list of detections
    def collision_detect(self, event):
        try:
            assert type(event) == carla.libcarla.Collision.Event
        except Exception:
            print(traceback.format_exc())

        collisions_len = len(self.__collisions)
        # other impulse is a change in momentum - indicates magnitute and direction in global coordinates
        collision = (event.actor, event.other_actor, event.normal_impulse)
        self.__collisions.append(collision)

        # post condition
        try:
            assert len(self.__collisions) == collisions_len + 1
        except Exception:
            print(traceback.format_exc())
        
    # listen to sensor
    def listen(self):
        self.get_sensor().listen(lambda event: self.collision_detect(event))


"""
===========
Navigation Class

__init__(self, start, destination, world) creates instance and initilizes attributes
    self.__start = the inital location of the vechicle
    self.__destination = the coorinates the car wants to go to 
    self.__global_route_planner = the global route planner object
    self.__route = set of waypoints (the route)
   

getters for each of these attributes

lane_invasion(self, event) | adds collision (parent actor which invaded lane, line markings which were crossed) to self.__lane_invasions

listen(self) | retreives data from sensor and calls the lane_invasion method
===========
"""

class Navigation():
    def __init__(self, start, destination, world):
        self.__start = start.location
        self.__destiniation = destination.location
        self.__global_route_planner = GlobalRoutePlanner(world.get_map(), 1)
        self.__route = self.__global_route_planner.trace_route(self.__start, self.__destiniation)
        self.__current_waypoint_num = 5
        self.__current_waypoint = self.__route[self.__current_waypoint_num]

        # visualizing waypoints
        for waypoint in self.__route:
            world.debug.draw_string(waypoint[0].transform.location, '^', draw_shadow=False,
            color=carla.Color(r=0, g=0, b=255), life_time=90.0,
            persistent_lines=True)
    
    def get_start(self):
        return self.__start
    def get_destination(self):
        return self.__destiniation
    def get_global_route_planner(self):
        return self.__global_route_planner
    def get_route(self):
        return self.__route
    def get_cur_waypoint(self):
        return self.__current_waypoint
    def get_cur_waypoint_num(self):
        return self.__current_waypoint_num
    def set_cur_waypoint(self, num):
        self.__current_waypoint = self.__route[num]
        self.__current_waypoint_num = num
    def advance_waypoint(self, car):
        while self.__current_waypoint_num < len(self.__route) and car.get_transform().location.distance(self.__route[self.__current_waypoint_num][0].transform.location)<5:
            self.__current_waypoint_num +=1
       
    
    # angle between two vectors
    def angle_between(self, v1, v2):
        return math.degrees(np.arctan2(v1[1], v1[0]) - np.arctan2(v2[1], v2[0]))

    # function to get angle between the car and target waypoint
    def get_angle(self,car,wp):
        vehicle_pos = car.get_transform()
        car_x = vehicle_pos.location.x
        car_y = vehicle_pos.location.y
        wp_x = wp.transform.location.x
        wp_y = wp.transform.location.y
        
        # vector to waypoint
        x = (wp_x - car_x)/((wp_y - car_y)**2 + (wp_x - car_x)**2)**0.5
        y = (wp_y - car_y)/((wp_y - car_y)**2 + (wp_x - car_x)**2)**0.5
        
        #car vector
        car_vector = vehicle_pos.get_forward_vector()
        degrees = self.angle_between((x,y),(car_vector.x,car_vector.y))

        corrected_deg = self.correct_angle(degrees)
        return corrected_deg
    
    def get_proper_angle(self, car,wp_idx,rte):
        # create a list of angles to next 5 waypoints starting with current
        next_angle_list = []
        for i in range(10):
            if wp_idx + i*3 <len(rte)-1:
                next_angle_list.append(self.get_angle(car,rte[wp_idx + i*3][0]))
        idx = 0
        while idx<len(next_angle_list)-2 and abs(next_angle_list[idx])>40:
            idx +=1
        return wp_idx+idx*3,next_angle_list[idx]
    
    def correct_angle(self, degrees):
        if degrees<-300:
            fixed_deg = degrees +360
        elif degrees> 300:
            fixed_deg = degrees -360
        else:
            fixed_deg = degrees

         # limit steering to max angle 40 degrees
        if fixed_deg <-50:
            steer_input = -50
        elif fixed_deg> 50:
            steer_input = 50
        else:
            steer_input = fixed_deg
        
        return steer_input

#initialize the list of actors
def init_actors(spawn, blueprint_lib, spts, world):
    nav = Navigation(spawn, spts[6], world.get_world())
    vehicle = Vehicle(blueprint_lib, world.get_world(), spawn, nav)
    car = vehicle.get_car()
    transforms = [carla.Transform(carla.Location(x=2.8, z=0.7)), carla.Transform(carla.Location(x=4.8, z=0.7)), carla.Transform(carla.Location(x=6.8, z=0.7))]
    blueprints = [blueprint_lib.find('sensor.other.obstacle'), blueprint_lib.find('sensor.other.collision'), blueprint_lib.find('sensor.other.lane_invasion')]
    blueprints[0].set_attribute('distance', '20.0')
    vehicle.set_sensors(transforms, car, blueprints, world, blueprint_lib)

    return (vehicle, car)

#repeating logic performed in the main function
def main_loop(spectator, car, vehicle):
    # Move the spectator behind the vehicle
    transform = carla.Transform(car.get_transform().transform(carla.Location(x=-4,z=2.5)),car.get_transform().rotation)
    spectator.set_transform(transform)
    time.sleep(0.005)
    if(vehicle.control_loop() == False):
        raise KeyboardInterrupt()

def clear_world(client):
    client.reload_world()
    print("World cleared :)\n")

def main():   
    world = World()
    client, map, spts, spawn, blueprint_lib = world.init_world()

    try:
        spectator = world.init_spectator(spawn)

        vehicle, car = init_actors(spawn, blueprint_lib, spts, world)

        while True:
            #try:
            main_loop(spectator, car, vehicle)
            #except KeyboardInterrupt:
            #    try:
            #        clear_world(client)
            #    finally:
            #        break
    except Exception:
        print(traceback.format_exc())
    except KeyboardInterrupt:
        print("Keyboard Interrupt")
    finally:
        clear_world(client)

if __name__ == "__main__":
    main()
