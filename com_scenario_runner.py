import logging
import os
import multiprocessing
import argparse
import time
import json
import signal

from carla_run_vehicle import commu_apollo_and_carla
from scenario_runner import scenario_run
from scenario_parser import ScenarioConfigurationParser as SrCfgP


class ProcessManage(object):
    def __init__(self, *argv):
        self._pro_pid_list = []
        for arg in argv:
            self._pro_pid_list.append(arg)
            logging.info("Add new pro pid={}".format(arg.pid))

        # pass
        signal.signal(signal.SIGINT, self._signal_handler)  # Ctrl + C
        signal.signal(signal.SIGTERM, self._signal_handler)

    def add_new_pro(self, *argv):
        for arg in argv:
            self._pro_pid_list.append(arg)

    def destroy(self):
        for pid in self._pro_pid_list:
            if pid is not None and pid.is_alive():
                pid.terminate()

    def _signal_handler(self, signum, frame):
        print("Receive signal. signum={}, frame={}", signum, frame)
        self.destroy()


def get_args():
    argparser = argparse.ArgumentParser(
        description='Run scenarios which run in carla, '
        'and the AD statck is Apollo')

    argparser.add_argument(
        '--carla-host',
        default='127.0.0.1',
        help='carla server ip addr (default: 127.0.0.1)')
    argparser.add_argument(
        '--carla-port',
        default=2000,
        type=int,
        help='carla port to connect to (default: 2000)')
    argparser.add_argument(
        '--apollo-host',
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
        help='ego vehicle role name')
    argparser.add_argument(
        '--show',
        action='store_true',
        help='display adc in pygame')
    argparser.add_argument(
        '--timeout',
        type=float,
        default=20.0,
        help='Set the CARLA client timeout value in seconds')
    argparser.add_argument(
        '--sensor-config',
        default="sensor_configs/apollo_600_modular_testing.json",
        help='specify sensor config file path')
    argparser.add_argument(
        '--log-dir',
        default="./logs",
        help='where to store log files')
    argparser.add_argument('--configFile',
                           default='./scenario_configs/'
                           '743_borrow_lane_cfg.json',
                           help='Provide a scenario configuration file '
                           '(*.json)')

    args = argparser.parse_args()

    ac_args = argparse.Namespace()  # communication with apollo and carla args
    ac_args.carla_host = args.carla_host
    ac_args.carla_port = args.carla_port
    ac_args.apollo_host = args.apollo_host
    ac_args.apollo_port = args.apollo_port
    ac_args.adc = args.adc
    ac_args.show = args.show
    ac_args.timeout = args.timeout
    ac_args.sensor_config = args.sensor_config
    ac_args.log_dir = args.log_dir

    # sr_host_keys = ['host', 'port', 'timeout']
    sr_host_dict = dict()
    sr_host_dict.update({'host': args.carla_host})
    sr_host_dict.update({'port': args.carla_port})
    sr_host_dict.update({'timeout': args.timeout})

    scenario_configurations = []
    # cfg_file = "./scenario_configs/{}".format(args.configFile)
    with open(args.configFile) as config_file:
        if config_file is not None:
            sr_config = json.load(config_file)
            sr_config.update({'list': False})
            scenario_configurations = \
                SrCfgP.parse_scenario_configuration_with_customer_param(
                        sr_config['scenario'],
                        sr_config['configFile'])
            sr_config.update(
                {'scenario_configurations': scenario_configurations})
        else:
            sr_config = None
            print("Error!!! Can't find the scenario config file.")

    if sr_config is not None:
        if len(scenario_configurations) > 0:
            ac_args.dst_x = scenario_configurations[0].destination.x
            ac_args.dst_y = scenario_configurations[0].destination.y
            ac_args.dst_z = scenario_configurations[0].destination.z

        sr_args = {**sr_host_dict, **sr_config}   # scenario runner args
        return ac_args, argparse.Namespace(**sr_args)
    else:
        return ac_args, None


def main():
    try:
        run_scenario = None
        commu_ac = None

        logging.info("Com scenario runner pid={}".format(os.getpid()))
        ac_args, sr_args = get_args()
        if sr_args is None:
            return

        run_scenario = multiprocessing.Process(target=scenario_run,
                                               args=(sr_args,))
        run_scenario.start()
        commu_ac = multiprocessing.Process(target=commu_apollo_and_carla,
                                           args=(ac_args,))
        commu_ac.start()

        ProcessManage(run_scenario, commu_ac)

        while True:
            if not run_scenario.is_alive():
                os.kill(commu_ac.pid, signal.SIGUSR1)
                break
            time.sleep(1)

    finally:
        pass


if __name__ == "__main__":
    main()
