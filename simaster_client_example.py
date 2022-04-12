from grpc_simaster.client import SimasterAPI
from utils import load_json


if __name__ == "__main__":
    config = load_json("./simaster_config.json")
    grpc_host = config['host']
    grpc_port = config['port']

    server = SimasterAPI(grpc_host, grpc_port)
    # get simaster version
    print(f"simaster version: {server.GetVersion()}")

    # run simulation
    # task_id = 10096
    # scenario_config_id = "scenario_config.781_stop_at_fix_location_cfg"
    # sensor_config_id = "sensor_config.apollo_600_modular_testing"
    # apollo_config_id = "apollo_config.pnc_testing"
    # print(server.Run(task_id, scenario_config_id, sensor_config_id, apollo_config_id))
