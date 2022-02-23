#!/usr/bin/python

import os
import fnmatch
import argparse
import logging
import json

from main import external_run_scenario
from http_client import Client

httpd_ip = '172.17.0.27'


def query_avaliable_apollo_carla():
    res = dict()
    res['apollo_host_ip'] = '172.17.0.11'
    res['apollo_host_port'] = 9090
    res['dreamview_port'] = 8888
    res['carla_host_ip'] = '172.17.0.2'
    res['carla_host_port'] = 2000

    return res


def find_out_all_scenarios():
    dir = './scenario_configs'
    all_file_with_path = []
    for file in fnmatch.filter(os.listdir(dir), '*.json'):
        file_with_path = os.path.join(dir, file)
        if os.path.getsize(file_with_path) > 0:
            all_file_with_path.append(file_with_path)

    return all_file_with_path


def run_specific_scenario(depend_service, scenario_cfg):
    args = argparse.Namespace()
    args.carla_host = depend_service['carla_host_ip']
    args.carla_port = depend_service['carla_host_port']
    args.apollo_host = depend_service['apollo_host_ip']
    args.apollo_port = depend_service['apollo_host_port']
    args.dreamview_port = depend_service['dreamview_port']
    args.apollo_config = './apollo_configs/pnc_testing.json'
    args.adc = 'hero'
    args.show = False
    args.timeout = 10
    args.sensor_config = './sensor_configs/apollo_600_modular_testing.json'
    args.configFile = scenario_cfg
    args.log_dir = './logs'
    args.fps = 50
    external_run_scenario(args)


def upload_scenario_result_report(client):
    dir = './'
    all_file_with_path = []
    for file in fnmatch.filter(os.listdir(dir), '*.json'):
        file_with_path = os.path.join(dir, file)
        if os.path.getsize(file_with_path) > 0:
            all_file_with_path.append(file_with_path)

    for scenario_result_report in all_file_with_path:
        with open(scenario_result_report, encoding='utf-8') as f:
            data = json.load(f)
            data_str = json.dumps(data)
            file_name = os.path.basename(scenario_result_report)
            logging.info("Ready to put %s to httpd", file_name)
            client.do_put(file_name, data_str)
        os.remove(scenario_result_report)


def auto_scenario_run():
    logging.basicConfig(level=logging.INFO)
    all_scenarios = find_out_all_scenarios()
    http_client = Client(httpd_ip)
    for scenario in all_scenarios:
        available_res = query_avaliable_apollo_carla()
        run_specific_scenario(available_res, scenario)
        upload_scenario_result_report(http_client)
    http_client.close()


if __name__ == "__main__":
    auto_scenario_run()
