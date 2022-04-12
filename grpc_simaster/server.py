import os
import time
import grpc
import logging
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
            centre_endpoint: str,
            log_dir: str) -> None:
        super().__init__()
        self.simulator_version = simulator_version
        self.db_user = db_user
        self.db_password = db_password
        self.db_host = db_host
        self.db_port = db_port
        self.database = database
        self.centre_endpoint = centre_endpoint
        self.log_dir = log_dir

    def Run(self, request: TaskArgs, reply: TaskReply):
        task_id = request.task_id
        scenario_config_id = request.scenario_config_id
        sensor_config_id = request.sensor_config_id
        apollo_config_id = request.apollo_config_id
        logging.info(
            f"receive new task: {task_id}#{scenario_config_id}"
            f"#{sensor_config_id}#{apollo_config_id}")

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
                apollo_config_id=apollo_config_id,
                log_dir=self.log_dir
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
            centre_endpoint: str,
            log_dir: str):
    if not os.path.isdir(log_dir):
        os.makedirs(log_dir)
    timestr = time.strftime("%Y%m%d-%H%M%S")
    logfilepath = os.path.join(log_dir, f"simaster.{timestr}.log")
    if os.path.isfile(logfilepath):
        os.remove(logfilepath)
    root = logging.Logger.root
    for h in root.handlers[:]:
        root.removeHandler(h)
        h.close()
    logging.basicConfig(
            filename=logfilepath,
            level=logging.INFO,
            format='%(asctime)s %(message)s')

    endpoint = f"{grpc_host}:{grpc_port}"
    db_endpoint = f"{db_host}:{db_port}"
    nacos_endpoint = centre_endpoint
    logging.info("starting grpc server")
    logging.info(f"version: {simulator_version} at {endpoint}")
    logging.info(f"using db at {db_endpoint}")
    logging.info(f"using nacos at {nacos_endpoint}")

    server = grpc.server(
        futures.ThreadPoolExecutor(
            max_workers=num_con))
    servicer = SimasterServicer(
        simulator_version=simulator_version,
        db_user=db_user,
        db_password=db_password,
        db_host=db_host,
        db_port=db_port,
        database=database,
        centre_endpoint=centre_endpoint,
        log_dir=log_dir)
    add_SimulationServicer_to_server(servicer, server)

    server.add_insecure_port(endpoint)
    server.start()
    try:
        logging.info("simaster is now running")
        print("Press <ctrl+c> to quit")
        while True:
            time.sleep(60 * 60 * 24)
    except KeyboardInterrupt:
        server.stop(0)
