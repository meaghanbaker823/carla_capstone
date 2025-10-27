# set up world - Arin

# Set up car - Calypso 

# move car - meaghan 

# sensors - Abby
import carla
import random
import time
import numpy as np
import cv2

IM_WIDTH = 640
IM_HEIGHT = 480

def process_img(image):
    i = np.array(image.raw_data)
    i2 = i.reshape((IM_HEIGHT, IM_WIDTH, 4))
    i3 = i2[:, :, :3]
    cv2.imshow("",i3)
    cv2.waitKey(1)
    return i3/ 255.0


client = carla.Client('localhost', 2000)

world = client.load_world('/Game/Carla/Maps/Town01')
world.set_weather(carla.WeatherParameters.ClearNoon)

blueprint_lib = world.get_blueprint_library()

actor_list = []

vehicle_bp = random.choice(blueprint_lib.filter('vehicle.bmw.*'))
camera_bp = blueprint_lib.find('sensor.camera.rgb')

spts = world.get_map().get_spawn_points()
spawn = spts[0]
spectator = world.get_spectator()
spec_trans = spectator.get_transform()
spectator.set_transform(carla.Transform(carla.Location(x = spawn.location.x, y = spawn.location.y, z = spawn.location.z + 60)))

# transform = carla.Transform(carla.Location(x = spawn.x, y = spawn.y, z = spawn.z), carla.Rotation(yaw=180))
car = world.spawn_actor(vehicle_bp, spawn)
actor_list.append(car)
# camera will be relative to car
vechicle_loc = car.get_location()

camera_bp.set_attribute("image_size_x", f"{IM_WIDTH}")
camera_bp.set_attribute("image_size_y", f"{IM_HEIGHT}")
camera_bp.set_attribute("fov", "110")
relative_transform = carla.Transform(carla.Location(x=2.5, z = 0.7))
camera = world.spawn_actor(camera_bp, relative_transform, attach_to=car)
actor_list.append(camera)

camera.listen(lambda data: process_img(data))
time.sleep(10)
for actor in actor_list:
    actor.destroy()
