import sys
import argparse
from grpc_simaster.server import serve_Simaster
from utils import load_json, get_simulator_version


def get_args():
    p = argparse.ArgumentParser("simaster grpc service")
    p.add_argument("--config", type=str,
                default="./simaster_config.json",
                help="path to configuration file")
    p.add_argument('-v', action='store_true',
                help="show version")
    return p.parse_args()


if __name__ == "__main__":
    args = get_args()

    version = get_simulator_version()
    if args.v:
        print(version)
        sys.exit(0)

    config = load_json(args.config)
    grpc_host = config['host']
    grpc_port = config['port']
    num_worker = config['num_worker']
    db_config = config['db']
    db_user = db_config['user']
    db_password = db_config['password']
    db_host = db_config['host']
    db_port = db_config['port']
    database = db_config['database']
    nacos_config = config['nacos']
    nacos_host = nacos_config['host']
    nacos_port = nacos_config['port']
    nacos_endpoint = f"{nacos_host}:{nacos_port}"

    serve_Simaster(
        num_con=num_worker,
        grpc_host=grpc_host,
        grpc_port=grpc_port,
        simulator_version=version,
        db_user=db_user,
        db_password=db_password,
        db_host=db_host,
        db_port=db_port,
        database=database,
        centre_endpoint=nacos_endpoint
    )
