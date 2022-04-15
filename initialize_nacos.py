import argparse
import json
import os

from service_discovery.nacos_utils import (
    publish_config,
    get_scenario_config)


SCENARIO_CONFIG_DIR = "./scenario_configs/"
SENSOR_CONFIG_DIR = "./sensor_configs/"
APOLLO_CONFIG_DIR = "./apollo_configs/"


def get_args():
    p = argparse.ArgumentParser()
    p.add_argument("host", help="nacos host ip")
    p.add_argument("--port", default="8848", help="nacos port")
    return p.parse_args()


def load_json(filepath: str) -> dict:
    with open(filepath) as f:
        data = json.load(f)
    return data


def load_file(filepath: str) -> str:
    data = load_json(filepath)
    return json.dumps(data)


def publish_scenario_configs(endpoint: str):
    files = os.listdir(SCENARIO_CONFIG_DIR)
    json_files  = []
    for f in files:
        if ".json" in f:
            json_files.append(f)
    print(f"all config files: {json_files}")

    for jf in json_files:
        filepath = os.path.join(SCENARIO_CONFIG_DIR, jf)
        json_dict = load_json(filepath)
        with open(json_dict['config']) as f:
            xml_str = f.read()

        filename = os.path.basename(filepath)
        key = f"scenario_config.{filename.split('.')[0]}"
        value_dict = {
            "scenario": json_dict['scenario'],
            "config_xml": xml_str,
        }
        if "loop" in json_dict:
            value_dict['loop'] = json_dict["loop"]
        # TODO
        # 把sumo配置发布到配置中心，目前由于配置过多，
        # 只把配置文件路径发布到配置中心
        if "sumo" in json_dict:
            # with open(json_dict['sumo']) as f:
            #     sumo_str = f.read()
            # value_dict['sumo_xml'] = sumo_str
            value_dict['sumo_cfg'] = json_dict['sumo']
        value = json.dumps(value_dict)
        if not publish_config(endpoint, key, value):
            print(f"fail to publish {filepath}")


def publish_sensor_configs(endpoint: str):
    files = os.listdir(SENSOR_CONFIG_DIR)
    for f in files:
        filepath = os.path.join(SENSOR_CONFIG_DIR, f)
        value = load_file(filepath)

        filename = os.path.basename(filepath)
        key = f"sensor_config.{filename.split('.')[0]}"

        isok = publish_config(endpoint, key, value)
        if not isok:
            print(f"fail to publish {filepath}")


def publish_apollo_configs(endpoint: str):
    files = os.listdir(APOLLO_CONFIG_DIR)
    for f in files:
        filepath = os.path.join(APOLLO_CONFIG_DIR, f)
        value = load_file(filepath)

        filename = os.path.basename(filepath)
        key = f"apollo_config.{filename.split('.')[0]}"

        isok = publish_config(endpoint, key, value)
        if not isok:
            print(f"fail to publish {filepath}")


if __name__ == "__main__":
    args = get_args()
    endpoint = f"{args.host}:{args.port}"

    publish_scenario_configs(endpoint)
    publish_sensor_configs(endpoint)
    publish_apollo_configs(endpoint)

    # test if configs really get pushed
    scenario_name, xml_tree, sumo_cfg, loop = \
        get_scenario_config(
            endpoint, "scenario_config.run_1000km")
    print(scenario_name)
    print(xml_tree)
    print(sumo_cfg)
    print(loop)
