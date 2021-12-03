import sys
import multiprocessing
import argparse
import time
import json

from carla_run_vehicle import commu_apollo_and_carla
from scenario_runner import scenario_run

# sys.path.append('/third-parties/scenario_runner/')

def get_args():    
    argparser = argparse.ArgumentParser(
        description='Run scenarios which run in carla, and the AD statck is Apollo')
    
    argparser.add_argument(
        '--carlaHost',
        default='127.0.0.1',
        help='carla server ip addr (default: 127.0.0.1)')
    argparser.add_argument(
        '--carlaPort',
        default=2000,
        type=int,
        help='carla port to connect to (default: 2000)')
    argparser.add_argument(
        '--apolloHost',
        default='127.0.0.1',
        help='apollo server ip addr (default: 127.0.0.1)')
    argparser.add_argument(
        '--apolloPort',
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
        '--list', action="store_true", help='List all supported scenarios and exit')    
    argparser.add_argument('--configFile', default='./scenario_configs/scenario_configs.json', help='Provide a scenario configuration file (*.json)')  

    args = argparser.parse_args()

    ac_args = argparse.Namespace()
    ac_args.carlaHost = args.carlaHost
    ac_args.carlaPort = args.carlaPort
    ac_args.apolloHost = args.apolloHost
    ac_args.apolloPort = args.apolloPort
    ac_args.adc = args.adc
    ac_args.show = args.show
    ac_args.timeout = args.timeout

    # sr_host_keys = ['host', 'port', 'timeout']
    sr_host_dict = dict()
    sr_host_dict.update({'host': args.carlaHost})
    sr_host_dict.update({'port': args.carlaPort})
    sr_host_dict.update({'timeout': args.timeout})

    with open(args.configFile) as sr_config_file:
        if sr_config_file is not None:            
            sr_config = json.load(sr_config_file)
            sr_config.update({'list': args.list})
        else:            
            sr_config = None 
            print("Error!!! Can't find the scenario config file.")

    if sr_config is not None:
        sr_args = {**sr_host_dict, **sr_config}
        return ac_args, argparse.Namespace(**sr_args)
    else:
        return ac_args, None

def main():
    try:
        run_scenario = None
        commu_ac = None

        ac_args, sr_args = get_args()
        if sr_args is None:
            return

        run_scenario = multiprocessing.Process(target=scenario_run, args=(sr_args,))
        run_scenario.start()
        if not sr_args.list:
            commu_ac = multiprocessing.Process(target=commu_apollo_and_carla, args=(ac_args,))
            commu_ac.start()
        
        run_scenario.join()
        if commu_ac is not None: commu_ac.join()
        
        # while True:
        #     if not run_scenario.is_alive():
        #         print("not alive")
        #         if commu_ac is not None:
        #             print("Ready to kill commu_ac process")
        #             commu_ac.terminate()
        #         break
        #     time.sleep(1)
        #     print("Parent process")
    
    finally:
       pass
       

if __name__ == "__main__":
    sys.exit(main())