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
    def __init__(self, world_map='town10HD_Opt'):
        self.__client = carla.Client('localhost', 2000)
        self.__client.set_timeout(30.0)

        cur_map = os.path.basename(self.__client.get_world().get_map().name)
        cur_map = cur_map[0].lower() + cur_map[1:]

        if world_map == cur_map:
            self.__world = self.__client.get_world()
        else:
            self.__world = self.__client.load_world(world_map)

        self.default_spawn = self.__world.get_map().get_spawn_points()[0]

    def get_client(self):
        return self.__client

    def get_world(self):
        return self.__world

    def change_map(self, world_map='town10HD_Opt'):
        return self.__world.load_world(world_map)

    def get_blueprints(self):
        return self.__world.get_blueprint_library()
    
    def get_spawnpoints(self):
        return self.__world.get_map().get_spawn_points()
    
# there are intersection entrance actors, stop sign actors, traffic light actors
#TODO: command list for Vehicle

class Vehicle: 
    def __init__(self, blueprint_lib, world_map, spawn, navigator):
        vehicle_bp = random.choice(blueprint_lib.filter('vehicle.bmw.*'))
        self.__car = world_map.spawn_actor(vehicle_bp, spawn)
        self.__actors = world_map.get_actors()
        self.__sensors = Sensors()
        self.__light_color = "unknown"
        self.__navigator = navigator
    
    
    def get_navigator(self):
        return self.__navigator
  
    def get_car(self):
        return self.__car
    
    def get_sensors(self):
        return self.__sensors

    def get_light_color(self):
        return self.__light_color
    
    def set_light_color(self, color):
        self.__light_color = color
    
    def set_sensors(self, transform, actor, blueprint, world):
        self.__sensors.add_sensor(ObstacleSensor(transform[0], actor, blueprint[0], world))
        self.__sensors.add_sensor(CollisionSensor(transform[1], actor, blueprint[1], world))
        self.__sensors.add_sensor(LaneInvasionSensor(transform[2], actor, blueprint[2], world))
    
        for sensor in self.__sensors.get_sensors():
            sensor.listen()

    def check_traffic_light(self, target_distance):
        traffic_light_near = False
        all_lights = self.__actors.filter('traffic.traffic_light*')
        for light in all_lights:

            car_location = self.get_car().get_location()
            light_location = light.get_location()

            distance = car_location.distance(light_location)

            car_rotation= int(self.get_car().get_transform().rotation.roll)
            light_rotation= int(light.get_transform().rotation.roll)

            rotation_difference = abs(car_rotation - light_rotation)

            if((rotation_difference < 30) and (distance < target_distance)):

                traffic_light_near = True
                color = light.get_state()

                old_color = self.get_light_color()

                if color == carla.TrafficLightState.Red:
                    self.set_light_color("red")
                elif color == carla.TrafficLightState.Yellow:
                    self.set_light_color("yellow")
                elif color == carla.TrafficLightState.Green:
                    self.set_light_color("green")
                else:
                    self.set_light_color("unknown")
                if(old_color != self.get_light_color()):
                    print("Traffic light is ", self.get_light_color())

        return traffic_light_near

    def control_loop(self):
        car_changed = False

        if(self.__sensors.get_sensors()[1].get_collisions() != []):
            self.stop_car()
            return False
        
        if(self.__sensors.get_sensors()[0].get_other_actors() != []):
            car_changed = True
            self.avoid_obstacles()

        if(self.check_traffic_light(10)):
            car_changed = True
            self.reactToTrafficLight(self.__light_color)

        if(self.__sensors.get_sensors()[2].get_lane_markings() != []):
            car_changed = True
            self.fix_lane()

        if(not car_changed):
            self.drive()

    def is_car_moving(self):
        if(self.get_car().get_velocity() == 0):
            return False
        else:
            return True

    def maintain_speed(self, s):
        PREFERRED_SPEED = 30        # targeted speed in kph
        SPEED_THRESHOLD = 2         # how many kph we can comfortably be under the target
        if s >= PREFERRED_SPEED:
            return 0
        elif s < PREFERRED_SPEED - SPEED_THRESHOLD:
            return 0.8      # essentially 80% of gas
        else:   
            return 0.4      
        
    def drive(self):  
        self.__navigator.advance_waypoint(self.__car)
        v = self.get_car().get_velocity()                                   # velocity is a 3d vector in m/s
        speed = round(3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2),0)          # speed in kilometers/hr 
        waypt_nm, steering_angle = self.__navigator.get_proper_angle(self.__car, self.__navigator.get_cur_waypoint_num(), self.__navigator.get_route()) 
        self.__navigator.set_cur_waypoint(waypt_nm)
        steering_angle /= 75
        estimated_throttle = self.maintain_speed(speed)
        self.get_car().apply_control(carla.VehicleControl(throttle=estimated_throttle,steer=steering_angle))
    
    def decelerate(self):
        self.get_car().apply_control(carla.VehicleControl(throttle=0, brake=0.7))

    def swerve_left(self):
        self.get_car().apply_control(carla.VehicleControl(throttle=0.7,steer=-1.0))

    def swerve_right(self):
        self.get_car().apply_control(carla.VehicleControl(throttle=0.4,steer=1.0))

    def avoid_obstacles(self):
        print("Car avoiding ", self.__sensors.get_sensors()[0].get_other_actors()[0].type_id)
        self.swerve_left()
        # what is the obstacle and then go from there
        self.__sensors.get_sensors()[0].delete_old_detection()
 
    def stop_car(self):
        if(self.is_car_moving):
            self.get_car().apply_control(carla.VehicleControl(throttle=0,steer=0,brake=0.5))

    def reactToTrafficLight(self, color):
        if(color == "red"):
            self.stop_car()
        elif(color == "yellow"):
            self.stop_car()
        else:
            self.drive()

    #ARIN
    def fix_lane(self):
        return True


"""
===========
Sensors Class

__init__ creates instance and creates list to manage all sensors

get_sensors()                    | returns list of sensors

add_sensor(sensor: Carla sensor) | adds sensor to list of sensors

destroy_sensors()                | destroys all sensors in sensor list

===========
"""
class Sensors:
    def __init__(self):
        self.__sensors = []

    def get_sensors(self):
        return self.__sensors
    
    def add_sensor(self, sensor):
        self.__sensors.append(sensor)

    def destroy_sensors(self):
        print(f"{len(self.__sensors)}heloooooooo")
        for sensor in self.__sensors:
            print("Sensor destroyed.")
            print(sensor)
            sensor.destroy()

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

class ObstacleSensor(Sensors):
    def __init__(self, relative_transform, parent_actor, blueprint, world):
        Sensors.__init__(self)
        self.__transform = relative_transform
        self.__parent = parent_actor
        self.__sensor = world.spawn_actor(blueprint, relative_transform, attach_to=self.__parent)
        self.__other_actors = []
        self.__detections = []
    
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
    def get_detections(self):
        return self.__detections
    def get_other_actors(self):
        return self.__other_actors
    
    def delete_old_detection(self):
        self.__detections.pop(0)
        self.__other_actors.pop(0)
          
    # with event, add to list of detections
    def obstacle_detect(self, event):
        if event.other_actor not in self.__other_actors:
            detection = (event.other_actor, event.distance)
            self.__detections.append(detection)
            self.__other_actors.append(event.other_actor)

    
    # listen to sensor
    def listen(self):
        self.__sensor.listen(lambda event: self.obstacle_detect(event))

    
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

class CollisionSensor(Sensors):
    def __init__(self, relative_transform, parent_actor, blueprint, world):
        Sensors.__init__(self)
        self.__transform = relative_transform
        self.__parent = parent_actor
        self.__sensor = world.spawn_actor(blueprint, relative_transform, attach_to=self.__parent)
        self.__collisions = []
    
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
    def get_collisions(self):
        return self.__collisions
    
    # with event, add to list of detections
    def collision_detect(self, event):
        # other impulse is a change in momentum - indicates magnitute and direction in global coordinates
        collision = (event.actor, event.other_actor, event.normal_impulse)
        self.__collisions.append(collision)
        
    # listen to sensor
    def listen(self):
        self.__sensor.listen(lambda event: self.collision_detect(event))

"""
===========
LaneInvasionsSensor Class

__init__(self, relative_transform, parent_actor, blueprint, world) creates instance and initilizes attributes
    self.__transform = where the sensor is in relation to parent
    self.__parent = the Carla object the sensor is attached to
    self.__sensor = the Carla sensor being created
    self.__lane_invasions = list of all lane invasions by this sensor

getters for each of these attributes

lane_invasion(self, event) | adds collision (parent actor which invaded lane, line markings which were crossed) to self.__lane_invasions

listen(self) | retreives data from sensor and calls the lane_invasion method
===========
"""

class LaneInvasionSensor(Sensors):
    def __init__(self, relative_transform, parent_actor, blueprint, world):
        Sensors.__init__(self)
        self.__transform = relative_transform
        self.__parent = parent_actor
        self.__sensor = world.spawn_actor(blueprint, relative_transform, attach_to=self.__parent)
        self.__lane_invasions = []
        self.__lane_markings = []
    
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
    def get_lane_invasions(self):        
        return self.__lane_invasions
    def get_lane_markings(self):
        return self.__lane_markings
    

    def delete_old_detection(self):
        self.__lane_invasions.pop(0)
        self.__lane_markings.pop(0)
    
    # with event, add to list of detections
    def lane_invasion(self, event):
        # actor is actor that sensor is attached to and invaded another lane
        #THIS LINE HAS A RUNTIME ERROR (TRYING TO OPERATE ON A DESTROYED ACTOR)
        lane_invasion = (event.actor, event.crossed_lane_markings)
        self.__lane_invasions.append(lane_invasion)
        self.__lane_markings.append(event.crossed_lane_markings)
    
    # listen to sensor
    def listen(self):
        self.__sensor.listen(lambda event: self.lane_invasion(event))

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
            color=carla.Color(r=0, g=0, b=255), life_time=30.0,
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
        if fixed_deg <-40:
            steer_input = -40 
        elif fixed_deg> 40 :
            steer_input = 40 
        else:
            steer_input = fixed_deg
        
        return steer_input
        
     

    
def main ():

    #initializing the world and spawns

    world = World('/Game/Carla/Maps/Town01')
    client = world.get_client()
    map = world.get_world()
    spts = world.get_spawnpoints()
    spawn = spts[0]
    blueprint_lib = world.get_blueprints()


    # Spectators
    spectator = map.get_spectator()
    spec_trans = spectator.get_transform()
    spectator.set_transform(carla.Transform(carla.Location(
        x = spawn.location.x, y = spawn.location.y, z = spawn.location.z + 60)))

    # actor list
    actor_list = []
    nav = Navigation(spawn, spts[6], map) 
    vehicle = Vehicle(blueprint_lib, map, spawn, nav)
    car = vehicle.get_car()
    actor_list.append(car)
    transforms = [carla.Transform(carla.Location(x=2.8, z=0.7)), carla.Transform(carla.Location(x=4.8, z=0.7)), carla.Transform(carla.Location(x=6.8, z=0.7))]
    blueprints = [blueprint_lib.find('sensor.other.obstacle'), blueprint_lib.find('sensor.other.collision'), blueprint_lib.find('sensor.other.lane_invasion')]
    blueprints[0].set_attribute('distance', '20.0')
    vehicle.set_sensors(transforms, car, blueprints, map)
    actor_list.append(vehicle.get_sensors().get_sensors()[0])
    actor_list.append(vehicle.get_sensors().get_sensors()[1])
    actor_list.append(vehicle.get_sensors().get_sensors()[2])




    while True:
        try:
            # Move the spectator behind the vehicle 
            transform = carla.Transform(car.get_transform().transform(carla.Location(x=-4,z=2.5)),car.get_transform().rotation) 
            spectator.set_transform(transform) 
            time.sleep(0.005)
            vehicle.drive()
            if(vehicle.control_loop() == False):
                raise KeyboardInterrupt()

        except KeyboardInterrupt:
            try: 
                client.reload_world()
                print("World cleared :)\n")
  
            finally:
                break
            

main()
