"""
    ==========================================
    Camera(parent, res_x, res_y, world: World)

    ==========================================
"""
class Camera:
    def __bounding_box(self):
        bound_x = 0.5 + self.__parent.bounding_box.extent.x
        bound_y = 0.5 + self.__parent.bounding_box.extent.y
        bound_z = 0.5 + self.__parent.bounding_box.extent.z

        return [
            (carla.Transform(carla.Location(x=-2.0*bound_x, y=0.0*bound_y, z=2.0*bound_z), carla.Rotation(pitch=8.0)), attachment.SpringArmGhost),
            (carla.Transform(carla.Location(x=0.8*bound_x, y=0.0*bound_y, z=1.3*bound_z)), attachmnet.Rigid),
            (carla.Transform(carla.Location(x=1.9*bound_x, y=1.0*bound_y, z=1.2*bound_z)), attachmnet.SpringArmGhost),
            (carla.Transform(carla.Location(x=-2.8*bound_x, y=0.0*bound_y, z=4.6*bound_z), carla.Rotation(pitch=6.0)), attachment.SpringArmGhost),
            (carla.Transform(carla.Location(x=-1.0, y=-1.0*bound_y, z=0.4*bound_z)), attachment.Rigid)
        ]

    def __setup_sensors(self):
        self.__sensors = [
            ["sensor.camera.rgb", cc.Raw, "Camera RGB"],
            ["sensor.camera.depth", cc.Raw, "Camera Depth (RAW)"],
            ["sensor.camera.depth", cc.Depth, "Camera Depth (Greyscale)"],
            ["sensor.camera.depth", cc.LogarithmicDepth, "Camera Depth (Logarithmic GreyScale)"],
            ["sensor.camera.semantic_segmentation", cc.Raw, "Camera Semantic Segmentation"],
            ["sensor.camera.semantic_segmentation", cc.CityScapesPallette]
        ]

        blp_library = self.__world.get_blueprints()

        for sensor in self.__sensors:
            blp = blp_library.find(item[0])

            if sensor[0].startswith("sensor.camera"):
                blp.set_attribute("image_size_x", self.__dim[0])
                blp.set_attribute("image_size_y", self.__dim[1])
            elif sensor[0].startswith("sensor.lidar"):
                blp.set_attribute("range", "50")
            sensor.append(blp)
    
    def __init__(self, parent, res_x=800, res_y=400, world):
        self.__parent = parent
        self.__sensor = None
        self.__surface = None
        self.__recording = False
        self.__dim = [res_x, res_y]
        self.__world = world

        self.__camera_transforms = self.__bounding_box()

        self.__setup_sensors()

        self.__transform_index = 1
        self.__index = None

    def toggle_camera(self):
        self.__transform_index = (self.__transform_index + 1) % len(self.__camera_transforms)
        self.set_sensor(self.__index)

    def set_sensor(self, index):
        index = index % len(self.__sensors)

        if self.__sensor is not None:
            self.__sensor.destroy()
            self.__surface = None

        self.__sensor = self.__world.get_world().spawn_actor(
            self.__sensors[index][-1],
            self.__camera_transform[self.__transform_index][0],
            attach_to=self.__parent,
            attachment_type=self.__camera_transform[self.__transform_index][1]
        )

        weak_self = weakref.ref(self)
        self.__sensor.listen(lambda image: CameraManager.__parse_image(weak_self, image))

        self.__index = index

    def get_dimension(self):
        return self.__dim

    def set_dimension(self, res_x=800, res_y=400):
        self.__dim = [res_x, res_y]

    def set_recording(self, n_state):
        self.__recording = n_state

    @staticmethod
    def __parse_image(weak_self, image):
        self = weak_self()

        if not self:
            return
        if self.__sensors[self.__index][0].startswith("sensor.lidar"):
            points = np.frombuffer(image.raw_data, dtype=np.dtype("f4"))
            points = np.reshape(points, (int(points.shape[0] / 4), 4))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self.__dim) / 100.0
            lidar_data += (0.5 * self.__dim[0], 0.5 * self.__dim[1])
            lidar_data = np.fabs(lidar_data)
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self.__dim[0], self.__dim[1], 3)
            lidar_img = np.zeros(lidar_img_size)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self.__surface = pygame.surfarray.make_surface(lidar_img)
        else:
            image.convert(self.__sensors[self.__index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.widht, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self.__surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        if self.recording:
            image.save_to_disk("_out/%08d" % image.frame)
