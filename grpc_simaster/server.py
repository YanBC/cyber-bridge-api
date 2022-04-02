import time
import grpc
from concurrent import futures
from grpc_simaster.simaster_pb2_grpc import (
    SimulationServicer, add_SimulationServicer_to_server)
from grpc_simaster.simaster_pb2 import (
    TaskArgs, TaskReply, GetVersionArgs, GetVersionReply)
from grpc_simaster.core import run_scenario
from db.error_codes import ErrorCodes


class SimasterServicer(SimulationServicer):
    def __init__(
            self,
            simulator_version: str,
            db_user: str,
            db_password: str,
            db_host: str,
            db_port: int,
            database: str,
            centre_endpoint: str) -> None:
        super().__init__()
        self.simulator_version = simulator_version
        self.db_user = db_user
        self.db_password = db_password
        self.db_host = db_host
        self.db_port = db_port
        self.database = database
        self.centre_endpoint = centre_endpoint

    def Run(self, request: TaskArgs, reply: TaskReply):
        task_id = request.task_id
        scenario_config_id = request.scenario_config_id
        sensor_config_id = request.sensor_config_id
        apollo_config_id = request.apollo_config_id

        err: ErrorCodes = run_scenario(
                task_id=task_id,
                simulator_version=self.simulator_version,
                db_user=self.db_user,
                db_password=self.db_password,
                db_host=self.db_host,
                db_port=self.db_port,
                database=self.database,
                centre_endpoint=self.centre_endpoint,
                scenario_config_id=scenario_config_id,
                sensor_config_id=sensor_config_id,
                apollo_config_id=apollo_config_id
        )

        reply = TaskReply(err_code=err.value)
        return reply

    def GetVersion(self, request: GetVersionArgs, reply: GetVersionReply):
        reply = GetVersionReply(v=self.simulator_version)
        return reply


def serve_Simaster(
            num_con: int,
            grpc_host: str,
            grpc_port: int,
            simulator_version: str,
            db_user: str,
            db_password: str,
            db_host: str,
            db_port: int,
            database: str,
            centre_endpoint: str):
    server = grpc.server(
        futures.ThreadPoolExecutor(max_workers=num_con),)
    servicer = SimasterServicer(
        simulator_version=simulator_version,
        db_user=db_user,
        db_password=db_password,
        db_host=db_host,
        db_port=db_port,
        database=database,
        centre_endpoint=centre_endpoint)
    add_SimulationServicer_to_server(servicer, server)

    server.add_insecure_port(f"{grpc_host}:{grpc_port}")
    server.start()
    try:
        print("Simaster is running")
        print("Press <ctrl+c> to quit")
        while True:
            time.sleep(60 * 60 * 24)
    except KeyboardInterrupt:
        server.stop(0)
