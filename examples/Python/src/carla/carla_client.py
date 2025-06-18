import carla
import weakref
import random
import pygame
from pygame.locals import K_ESCAPE
from pygame.locals import K_SPACE
from pygame.locals import K_a
from pygame.locals import K_d
from pygame.locals import K_s
from pygame.locals import K_w
import numpy as np
from abc import ABC, abstractmethod

VIEW_WIDTH = 1920//2
VIEW_HEIGHT = 1080//2
VIEW_FOV = 90
BB_COLOR = (248, 64, 24)


def process_image_data(image):
    """
    Processes the raw image data from the camera sensor.
    """
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    return array


class BasicClient(ABC):
    """
    Basic implementation of a synchronous client.
    """
    @abstractmethod
    def game_start(self):
        pass

    @abstractmethod
    def control(self, car):
        pass

    def __init__(self):
        self.client = None
        self.world = None
        self.camera = None
        self.car = None

        self.display = None
        self.image = None
        self.capture = True

    def camera_blueprint(self):
        """
        Returns camera blueprint.
        """
        camera_bp = self.world.get_blueprint_library().find("sensor.camera.rgb")
        camera_bp.set_attribute("image_size_x", str(VIEW_WIDTH))
        camera_bp.set_attribute("image_size_y", str(VIEW_HEIGHT))
        camera_bp.set_attribute("fov", str(VIEW_FOV))
        return camera_bp

    def set_synchronous_mode(self, synchronous_mode):
        """
        Sets synchronous mode.
        """
        settings = self.world.get_settings()
        settings.fixed_delta_seconds = 0.05
        settings.synchronous_mode = synchronous_mode
        self.world.apply_settings(settings)

    def setup_car(self):
        """
        Spawns actor-vehicle to be controlled.

        """
        car_bp = self.world.get_blueprint_library().filter("vehicle.*")[0]
        location = random.choice(self.world.get_map().get_spawn_points())
        self.car = self.world.spawn_actor(car_bp, location)

    def setup_imu(self):
        """
        Spawns actor-IMU sensor to be used to get IMU data.
        """
        imu_bp = self.world.get_blueprint_library().find("sensor.other.imu")
        imu_transform = carla.Transform(carla.Location(x=0.5, z=2.8))
        self.imu = self.world.spawn_actor(
            imu_bp, imu_transform, attach_to=self.car)
        weak_self = weakref.ref(self)
        self.imu.listen(lambda data: weak_self().process_imu_data(data))

    def setup_camera(self):
        """
        Spawns actor-camera to be used to render view.
        """
        camera_transform = carla.Transform(carla.Location(
            x=-5.5, z=2.8), carla.Rotation(pitch=-15))
        self.camera = self.world.spawn_actor(
            self.camera_blueprint(), camera_transform, attach_to=self.car)
        weak_self = weakref.ref(self)
        self.camera.listen(
            lambda image: weak_self().set_image(weak_self, image))

        calibration = np.identity(3)
        calibration[0, 2] = VIEW_WIDTH / 2.0
        calibration[1, 2] = VIEW_HEIGHT / 2.0
        calibration[0, 0] = calibration[1, 1] = VIEW_WIDTH / \
            (2.0 * np.tan(VIEW_FOV * np.pi / 360.0))
        self.camera.calibration = calibration

    def process_imu_data(self, data):
        """
        Processes and stores IMU data.
        """
        self.imu_data = data

    @staticmethod
    def set_image(weak_self, img):
        self = weak_self()
        if self.capture:
            self.image = img
            self.capture = False

    def render(self, display):
        """
        Renders the image on the display.
        """
        if self.image is not None:
            array = process_image_data(self.image)
            surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
            display.blit(surface, (0, 0))
            return array

    def game_step(self):
        """
        Simulates one step in the game, processing inputs and rendering the image.
        """
        self.world.tick()
        self.capture = True
        self.pygame_clock.tick_busy_loop(20)
        image = self.render(self.display)

        pygame.display.flip()
        pygame.event.pump()

        sensor_data = None
        if not self.image or hasattr(self, "imu_data"):
            sensor_data = [self.image, self.imu_data.accelerometer.x]

        self.control(self.car)

        return sensor_data


class SyncClient(BasicClient):
    def control(self, car):
        keys = pygame.key.get_pressed()
        if keys[K_ESCAPE]:
            return True

        control = car.get_control()
        control.throttle = 0
        if keys[K_w]:
            control.throttle = 1
            control.reverse = False
        elif keys[K_s]:
            control.throttle = 1
            control.reverse = True
        if keys[K_a]:
            control.steer = max(-1., min(control.steer - 0.05, 0))
        elif keys[K_d]:
            control.steer = min(1., max(control.steer + 0.05, 0))
        else:
            control.steer = 0
        control.hand_brake = keys[K_SPACE]

        car.apply_control(control)
        return False

    def game_start(self):
        """
        Initializes the game, setting up the client, car, camera, IMU, and display.
        """
        pygame.init()

        self.client = carla.Client("localhost", 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()

        self.setup_car()
        self.setup_camera()
        self.setup_imu()

        self.display = pygame.display.set_mode(
            (VIEW_WIDTH, VIEW_HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF)
        self.pygame_clock = pygame.time.Clock()

        self.set_synchronous_mode(True)
        vehicles = self.world.get_actors().filter("vehicle.*")


class AsyncClient(BasicClient):
    def control(self, car):
        control = car.get_control()
        control.throttle = 1
        control.steer = 1
        car.apply_control(control)
        return False

    def game_start(self):
        """
        Initializes the game, setting up the client, car, camera, IMU, and display.
        """
        pygame.init()

        self.client = carla.Client("172.23.112.1", 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()

        self.setup_car()
        self.setup_camera()
        self.setup_imu()

        self.display = pygame.display.set_mode(
            (VIEW_WIDTH, VIEW_HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF)
        self.pygame_clock = pygame.time.Clock()

        self.set_synchronous_mode(False)
        vehicles = self.world.get_actors().filter("vehicle.*")
