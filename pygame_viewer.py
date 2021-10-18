import time
import sys
try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_q
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')
import carla
from examples.manual_control import (
                World, HUD, CameraManager,
                CollisionSensor, LaneInvasionSensor, GnssSensor, IMUSensor,
                find_weather_presets, get_actor_display_name)
from utils import (
    get_vehicle_by_role_name,
    is_actor_exist
)


class KeyboardControl:
    def parse_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True
            if event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True

    @staticmethod
    def _is_quit_shortcut(key):
        """Shortcut for quitting"""
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)


class WorldSR(World):
    restarted = False

    def __init__(self, carla_world, hud, ego_name, player):
        self.world = carla_world
        self.actor_role_name = ego_name
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            raise RuntimeError
        self.hud = hud
        self.player = player
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.imu_sensor = None
        self.radar_sensor = None
        self.camera_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._gamma = 2.2
        self.restart()
        self.world.on_tick(hud.on_world_tick)
        self.recording_enabled = False
        self.recording_start = 0
        self.constant_velocity_enabled = False
        self.current_map_layer = 0
        self.map_layer_names = [
            carla.MapLayer.NONE,
            carla.MapLayer.Buildings,
            carla.MapLayer.Decals,
            carla.MapLayer.Foliage,
            carla.MapLayer.Ground,
            carla.MapLayer.ParkedVehicles,
            carla.MapLayer.Particles,
            carla.MapLayer.Props,
            carla.MapLayer.StreetLights,
            carla.MapLayer.Walls,
            carla.MapLayer.All
        ]

    def restart(self):

        if self.restarted:
            return
        self.restarted = True

        self.player_max_speed = 1.589
        self.player_max_speed_fast = 3.713

        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0

        self.player_name = self.player.type_id

        # Set up the sensors.
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.imu_sensor = IMUSensor(self.player)
        self.camera_manager = CameraManager(self.player, self.hud, self._gamma)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)

    def tick(self, clock):
        if len(self.world.get_actors().filter(self.player_name)) < 1:
            return False

        self.hud.tick(self, clock)
        return True


def view_game(
        ego_name: str,
        carla_host: str,
        carla_port: int,
        screen_width: int = 1280,
        screen_height: int = 720):

    client = carla.Client(carla_host, carla_port)
    client.set_timeout(4.0)
    sim_world = client.get_world()

    player, player_type = get_vehicle_by_role_name(
                        __name__, sim_world, ego_name)

    # initialize pygame
    pygame.init()
    pygame.font.init()
    world = None

    display = pygame.display.set_mode(
        (screen_width, screen_height),
        pygame.HWSURFACE | pygame.DOUBLEBUF
    )
    hud = HUD(screen_width, screen_height)
    world = WorldSR(client.get_world(), hud, ego_name, player)
    clock = pygame.time.Clock() # for client fps
    controller = KeyboardControl()

    try:
        while True:
            sim_world.wait_for_tick()
            if controller.parse_events():
                break

            if not is_actor_exist(sim_world, actor_type=player_type):
                break

            clock.tick()
            if not world.tick(clock):
                break
            world.render(display)
            pygame.display.flip()

    finally:
        # manually destroy all attached sensors
        # otherwise carla server might crash
        if world is not None:
            world.destroy()
        pygame.quit()
