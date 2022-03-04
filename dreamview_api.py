from websocket import create_connection
import json
import logging
import carla
from typing import List
from utils import longitudinal_offset
import time


class Connection:
    _SimControlStatus = None
    _HMIStatus = None
    _VehicleParam = None

    def __init__(
            self,
            ip: str,
            port: int = 8888):
        self.url = f"ws://{ip}:{port}/websocket"
        self.ws = create_connection(self.url)
        self._SimControlStatus = json.loads(self.ws.recv())
        self._HMIStatus = json.loads(self.ws.recv())
        self._VehicleParam = json.loads(self.ws.recv())

    ########################
    # basic
    ########################
    def reconnect(self):
        """
        Closes the websocket connection and re-creates it so that data can be received again
        """
        self.ws.close()
        self.ws = create_connection(self.url)
        return

    def refresh_status(self) -> None:
        self.reconnect()
        self._SimControlStatus = json.loads(self.ws.recv())
        self._HMIStatus = json.loads(self.ws.recv())
        self._VehicleParam = json.loads(self.ws.recv())
        return

    def __del__(self):
        self.ws.close()

    ########################
    # get
    ########################
    def get_SimControlStatus(self) -> dict:
        return self._SimControlStatus

    def get_HMIStatus(self) -> dict:
        return self._HMIStatus

    def get_VehicleParam(self) -> dict:
        return self._VehicleParam

    def get_all_maps(self) -> list:
        return self._HMIStatus["data"]["maps"]

    def get_all_modes(self) -> list:
        return self._HMIStatus["data"]["modes"]

    def get_all_modules(self) -> list:
        return list(self._HMIStatus["data"]["modules"].keys())

    def get_all_vehicles(self) -> list:
        return self._HMIStatus["data"]["vehicles"]

    # # There is a bug in Apollo Dreamview. This function
    # # returns incorrect information. Disabling it for now.
    # def is_module_up(self, module: str) -> bool:
    #     return self._HMIStatus["data"]["modules"].get(module, False)

    ########################
    # set
    ########################
    def set_destination(
            self,
            start_pos: carla.Waypoint,
            end_pos: carla.Waypoint,
            rear_to_center_in_x: float = -1.4224) -> None:
        '''
        rear_to_center_in_x is the offset between location of
        the car and the location of the gps sensor
        '''
        start_location = longitudinal_offset(start_pos, -3)
        end_location = end_pos.transform.location

        json_msg = json.dumps({
            "type": "SendRoutingRequest",
            "start": {
                "x": start_location.x + rear_to_center_in_x,
                "y": -start_location.y,
                "z": 0
            },
            "end": {
                "x": end_location.x + rear_to_center_in_x,
                "y": -end_location.y,
                "z": 0
            },
            "waypoint": []
        })
        self.ws.send(json_msg)
        return

    def enable_module(self, module: str) -> None:
        self.ws.send(
            json.dumps({"type": "HMIAction", "action": "START_MODULE", "value": module})
        )
        return

    def enable_modules(self, modules: List[str]) -> None:
        for m in modules:
            self.enable_module(m)
        return

    def disable_module(self, module: str) -> None:
        self.ws.send(
            json.dumps({"type": "HMIAction", "action": "STOP_MODULE", "value": module})
        )
        return

    def disable_modules(self, modules: List[str]) -> None:
        for m in modules:
            self.disable_module(m)
        return

    def set_map(self, map: str) -> None:
        self.ws.send(
            json.dumps({"type": "HMIAction", "action": "CHANGE_MAP", "value": map})
        )
        return

    def set_vehicle(self, vehicle) -> None:
        self.ws.send(
            json.dumps(
                {"type": "HMIAction", "action": "CHANGE_VEHICLE", "value": vehicle}
            )
        )
        return

    def set_mode(self, mode: str) -> None:
        self.ws.send(
            json.dumps({"type": "HMIAction", "action": "CHANGE_MODE", "value": mode})
        )
        return


def setup_apollo(
        apollo_ip: str,
        dreamview_port: str,
        dreamview_mode: str,
        hd_map: str,
        vehicle: str,
        apollo_modules: List[str],
        start_pos: carla.Waypoint,
        end_pos: carla.Waypoint) -> bool:
    conn = Connection(apollo_ip, dreamview_port)

    available_modes = conn.get_all_modes()
    available_maps = conn.get_all_maps()
    available_vehicles = conn.get_all_vehicles()
    if dreamview_mode not in available_modes:
        logging.error(f"{dreamview_mode} not available")
        return False
    if hd_map not in available_maps:
        logging.error(f"{hd_map} not available")
        return False
    if vehicle not in available_vehicles:
        logging.error(f"{vehicle} not available")
        return False
    conn.set_mode(dreamview_mode)
    conn.set_map(hd_map)
    conn.set_vehicle(vehicle)

    conn.refresh_status()
    available_modules = conn.get_all_modules()
    for m in apollo_modules:
        if m not in available_modules:
            logging.error(f"{m} not available in {dreamview_mode}")
            return False
    conn.enable_modules(apollo_modules)
    # make sure apollo modules are set up
    # before routing request is sent
    time.sleep(5)
    if end_pos is not None:
        conn.set_destination(start_pos, end_pos)
    return True


def reset_apollo(
        apollo_ip: str,
        dreamview_port: str,
        apollo_modules: List[str]) -> None:
    conn = Connection(apollo_ip, dreamview_port)
    conn.disable_modules(apollo_modules)
    return


if __name__ == '__main__':
    dreamview_ip = "172.17.0.23"
    dreamview_port = 8888
    carla_ip = "172.17.0.5"
    carla_port = 2000
    start_x = 78.0
    start_y = 193.1
    end_x = 121.4
    end_y = 193

    # cli = carla.Client(carla_ip, carla_port)
    # sim_world = cli.get_world()
    # map = sim_world.get_map()
    # start_waypoint = map.get_waypoint(
    #         carla.Location(x=start_x, y=start_y))
    # end_waypoint = map.get_waypoint(
    #         carla.Location(x=end_x, y=end_y))

    conn = Connection(dreamview_ip, dreamview_port)
    # conn.set_destination(start_waypoint, end_waypoint)

    print(f"Available Modes: {conn.get_all_modes()}")
    print(f"Available Maps: {conn.get_all_maps()}")
    print(f"Available Vehicles: {conn.get_all_vehicles()}")
    print(f"Available Modules: {conn.get_all_modules()}")
    # conn.set_map('Carla Town03')
    # conn.enable_module("Planning")
    # conn.disable_module("Planning")