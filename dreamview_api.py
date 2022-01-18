from websocket import create_connection
import json
import carla
from utils import longitudinal_offset


class Connection:
    def __init__(
            self,
            ip: str,
            port: int = 8888):
        self.url = f"ws://{ip}:{port}/websocket"
        self.ws = create_connection(self.url)
        self.sequence_num = 0

    def set_destination(
            self,
            start_pos: carla.Waypoint,
            end_pos: carla.Waypoint,
            rear_to_center_in_x: float = -1.4224) -> int:
        '''
        rear_to_center_in_x is the offset between location of
        the car and the location of the gps sensor

        return 190 if ok
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

        return self.ws.send(json_msg)

    def reconnect(self):
        """
        Closes the websocket connection and re-creates it so that data can be received again
        """
        self.ws.close()
        self.ws = create_connection(self.url)
        return


if __name__ == '__main__':
    dreamview_ip = "172.17.0.5"
    dreamview_port = 8888
    carla_ip = "172.17.0.3"
    carla_port = 2000
    start_x = 78.0
    start_y = 193.1
    end_x = 121.4
    end_y = 193

    cli = carla.Client(carla_ip, carla_port)
    sim_world = cli.get_world()
    map = sim_world.get_map()
    start_waypoint = map.get_waypoint(
            carla.Location(x=start_x, y=start_y))
    end_waypoint = map.get_waypoint(
            carla.Location(x=end_x, y=end_y))

    conn = Connection(dreamview_ip, dreamview_port)
    conn.set_destination(start_waypoint, end_waypoint)
