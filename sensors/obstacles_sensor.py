import carla
from sensors.base_sensor import Sensor
from modules.prediction.proto.prediction_obstacle_pb2 import PredictionObstacles


class ObstacleSensor(Sensor):
    _apollo_channel = '/apollo/prediction'
    _apollo_msgType = 'apollo.prediction.PredictionObstacles'
    _apollo_pbCls = PredictionObstacles

    def __init__(self, ego_vehicle: carla.Vehicle) -> None:
        super().__init__(ego_vehicle)

    def update(self):
        pass


class DummyObstacleSensor(Sensor):
    _apollo_channel = '/apollo/prediction'
    _apollo_msgType = 'apollo.prediction.PredictionObstacles'
    _apollo_pbCls = PredictionObstacles

    def update(self):
        self._pbCls = PredictionObstacles()
        self._pbCls.header.CopyFrom(self._get_cyber_header())
        self._updated = True
