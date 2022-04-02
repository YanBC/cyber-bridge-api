import grpc
from grpc_simaster.simaster_pb2 import (
    TaskArgs, TaskReply, GetVersionArgs, GetVersionReply)
from grpc_simaster.simaster_pb2_grpc import (
    SimulationStub)


class SimasterAPI:
    def __init__(self, host, port) -> None:
        channel = grpc.insecure_channel(f"{host}:{port}")
        self.stub = SimulationStub(channel)

    def Run(
            self,
            task_id: int,
            scenario_config_id: str,
            sensor_config_id: str,
            apollo_config_id: str) -> int:
        request = TaskArgs(
            task_id=task_id,
            scenario_config_id=scenario_config_id,
            sensor_config_id=sensor_config_id,
            apollo_config_id=apollo_config_id
        )
        reply: TaskReply = self.stub.Run(request)
        return reply.err_code

    def GetVersion(self) -> str:
        request = GetVersionArgs()
        reply: GetVersionReply = self.stub.GetVersion(request)
        return reply.v
