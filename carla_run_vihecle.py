import argparse
import time
import sys
import multiprocessing

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
from modules.control.proto.control_cmd_pb2 import ControlCommand
from decoders.apollo_control_decoder import ApolloControlDecoder
from cyber_bridge_client import CyberBridgeClient
from sensors.location_sensor import LocationSensor
from sensors.chassis_sensor import ChassisSensor
from sensors.trajectory_sensor import TrajectorySensor
from sensors.base_sensor import SensorManager
from sensors.apollo_control import listen_and_apply_control


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

    def __init__(self, carla_world, hud, args, player):
        self.world = carla_world
        self.actor_role_name = args.adc
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
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



def get_args():
    argparser = argparse.ArgumentParser(
        description='CARLA Automatic Control Client')
    argparser.add_argument(
        '--carla',
        default='127.0.0.1',
        help='carla server ip addr (default: 127.0.0.1)')
    argparser.add_argument(
        '--carla-port',
        default=2000,
        type=int,
        help='carla port to connect to (default: 2000)')
    argparser.add_argument(
        '--apollo',
        default='127.0.0.1',
        help='apollo server ip addr (default: 127.0.0.1)')
    argparser.add_argument(
        '--apollo-port',
        default=9090,
        type=int,
        help='apollo port to connect to (default: 9090)')
    argparser.add_argument(
        '--adc',
        default='hero',
        help='ego vehicle name')
    argparser.add_argument(
        '--show',
        default=True,
        type=bool,
        help='display adc in pygame')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='Window resolution (default: 1280x720)')
    argparser.add_argument(
        '--channel',
        default="/apollo/control",
        help="Apollo control channel name"
    )
    argparser.add_argument(
        '--msgtype',
        default="apollo.control.ControlCommand",
        help="Apollo control channel message type"
    )

    args = argparser.parse_args()
    if args.show:
        args.width, args.height = [int(x) for x in args.res.split('x')]
    return args


def emergency_stop():
    """
    Send an emergency stop command to the vehicle

        :return: control for braking
    """
    control = carla.VehicleControl()
    control.steer = 0.0
    control.throttle = 0.0
    control.brake = 1.0
    control.hand_brake = False
    return control


def main():
    args = get_args()

    if args.show:
        pygame.init()
        pygame.font.init()
        world = None

    try:
        client = carla.Client(args.carla, args.carla_port)
        client.set_timeout(4.0)

        sim_world = client.get_world()
        settings = sim_world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        sim_world.apply_settings(settings)

        player = None
        player_name = None
        while player is None:
            print("Waiting for the ego vehicle...")
            time.sleep(1)
            possible_vehicles = sim_world.get_actors().filter('vehicle.*')
            for vehicle in possible_vehicles:
                if vehicle.attributes['role_name'] == args.adc:
                    print("Ego vehicle found")
                    player = vehicle
                    player_name = player.type_id
                    break

        if args.show:
            display = pygame.display.set_mode(
                (args.width, args.height),
                pygame.HWSURFACE | pygame.DOUBLEBUF
            )
            hud = HUD(args.width, args.height)
            world = WorldSR(client.get_world(), hud, args, player)  # sensors setup?
            clock = pygame.time.Clock() # for client fps
        controller = KeyboardControl()

        apollo_host = args.apollo
        apollo_port = args.apollo_port
        # apollo_channel = args.channel
        # apollo_msgType = args.msgtype
        # ctrl_decoder = ApolloControlDecoder(
        #         ControlCommand,
        #         apollo_channel,
        #         apollo_msgType)
        # cb_client = CyberBridgeClient(
        #         apollo_host, apollo_port,
        #         [], [ctrl_decoder])
        # cb_client.initialize()

        # TODO: setup sensors here
        location_sensor = LocationSensor(player)
        chassis_sensor = ChassisSensor(player)
        trajectory_sensor = TrajectorySensor(player)
        sensor_manager = SensorManager(
                apollo_host, apollo_port,
                [location_sensor, chassis_sensor, trajectory_sensor])

        control_sensor = multiprocessing.Process(
                            target=listen_and_apply_control,
                            args=(args.adc, args.carla, args.carla_port, apollo_host, apollo_port))
        control_sensor.start()

        while True:
            sim_world.tick()

            if controller.parse_events():
                break

            if len(sim_world.get_actors().filter(player_name)) < 1:
                print("ego vehicle destroyed, stopping...")
                break

            if args.show:
                clock.tick()
                if not world.tick(clock):
                    break
                world.render(display)
                pygame.display.flip()

            # recv_list = cb_client.recv_pb_messages()
            # if len(recv_list) > 0:
            #     apollo_control = ctrl_decoder.protobufToCarla(recv_list[-1])
            #     apollo_control.manual_gear_shift = False
            # else:
            #     print("No control message received, applying emergency stop")
            #     apollo_control = emergency_stop()
            #     # print("No control message received, applying previous control")
            #     # apollo_control = player.get_control()
            # player.apply_control(apollo_control)

            sensor_manager.send_apollo_msgs()

    finally:
        settings = sim_world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        sim_world.apply_settings(settings)

        if args.show:
            if world is not None:
                world.destroy()
            pygame.quit()


if __name__ == '__main__':
    main()

