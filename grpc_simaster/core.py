import time
import datetime
import multiprocessing
import logging

from simulation import start_simulation
from service_discovery.nacos_utils import (
        acquire_servers,
        get_scenario_config,
        get_sensor_config,
        get_apollo_config
)
from db.error_codes import ErrorCodes
from db.mysql_utils import create_connection, save_result


def get_configs(
        endpoint, scenario_config_id,
        sensor_config_id, apollo_config_id):
    success = True
    scenario_name = ""
    xml_tree = None
    sensor_config = None
    apollo_config = None
    try:
        scenario_name, xml_tree = get_scenario_config(
                endpoint, scenario_config_id)
        sensor_config = get_sensor_config(
                endpoint, sensor_config_id)
        apollo_config = get_apollo_config(
                endpoint, apollo_config_id)
    except Exception as e:
        success = False
        logging.error(f"error in acquiring configs, {e}")
    finally:
        return success, scenario_name, xml_tree, sensor_config, apollo_config


def get_servers(endpoint):
    success = True
    stop_event = multiprocessing.Event()
    apollo_host = ""
    bridge_port = 9090
    dreamview_port = 8888
    carla_host = ""
    carla_port = 2000
    try:
        # TODO
        # duration is hardcoded to 5 mins because there is
        # no way to tell how long a simulation will run
        # for now.
        apollo_host, carla_host = acquire_servers(
            stop_event, endpoint, duration=60*5)
    except Exception as e:
        success = False
        logging.error(f"error in acquiring servers, {e}")
    finally:
        return success, stop_event, apollo_host, bridge_port, dreamview_port, carla_host, carla_port


def write_to_db(
        db_user,
        db_password,
        db_host,
        db_port,
        database,
        task_id,
        error_code: ErrorCodes,
        sensors_config_id,
        apollo_config_id,
        scenario_id,
        simulator_version,
        carla_version: str = None,
        apollo_version: str = None,
        carla_ip: str = None,
        carla_port: int = None,
        apollo_ip: str = None,
        dreamview_port: int = None,
        bridge_port: int = None,
        start_time: str = None,
        end_time: str = None,
        ego: str = None,
        criteria: list = []) -> bool:
    cnx = create_connection(
            user=db_user,
            password=db_password,
            host=db_host,
            port=db_port,
            database=database)
    e_code = error_code.value
    try:
        save_result(
            cnx,
            task_id,
            e_code,
            simulator_version,
            carla_version,
            apollo_version,
            carla_ip,
            carla_port,
            apollo_ip,
            dreamview_port,
            bridge_port,
            sensors_config_id,
            apollo_config_id,
            scenario_id,
            start_time,
            end_time,
            ego,
            criteria)
        cnx.commit()
        return True
    except Exception as e:
        logging.error(f"fail to write db, {e}")
        return False
    finally:
        cnx.close()


def create_timestamp() -> str:
    cur_time = time.time()
    timestamp = datetime.datetime.fromtimestamp(cur_time).strftime('%Y-%m-%d %H:%M:%S')
    return timestamp


def is_task_exist(cnx, task_id: int) -> bool:
    sql = "SELECT task_id FROM results WHERE task_id = %s"
    cursor = cnx.cursor()
    cursor.execute(sql, (task_id,))
    cursor.fetchone()
    num_entry = cursor.rowcount
    cursor.close()
    return num_entry != 0


def run_scenario(
        task_id: int,
        simulator_version: str,
        db_user: str,
        db_password: str,
        db_host: str,
        db_port: int,
        database: str,
        centre_endpoint: str,
        scenario_config_id: str,
        sensor_config_id: str,
        apollo_config_id: str) -> ErrorCodes:
    err_code: ErrorCodes = ErrorCodes.SUCCESS
    criteria = []

    # test db connection
    try:
        tmp_cnx = create_connection(
                user=db_user, password=db_password,
                host=db_host, port=db_port, database=database)
    except Exception as e:
        logging.error("fail to connect to db, {e}")
        err_code = ErrorCodes.NETWORK_ERROR_MYSQL
        return err_code

    task_exist = is_task_exist(tmp_cnx, task_id=task_id)
    tmp_cnx.close()
    del tmp_cnx

    if task_exist:
        logging.error(f"task already exists, task id: {task_id}")
        err_code = ErrorCodes.CONFIG_ERROR
        return err_code

    ############################################################
    # query services registry and configuration centre
    ############################################################
    stop_event = multiprocessing.Event()
    try:
        scenario_name, xml_tree = get_scenario_config(
                centre_endpoint, scenario_config_id)
        sensor_config = get_sensor_config(
                centre_endpoint, sensor_config_id)
        apollo_config = get_apollo_config(
                centre_endpoint, apollo_config_id)
        if xml_tree is None or \
                sensor_config is None or \
                apollo_config is None:
            err_code = ErrorCodes.CONFIG_ERROR
            ok = write_to_db(
                    db_user=db_user,
                    db_password=db_password,
                    db_host=db_host,
                    db_port=db_port,
                    database=database,
                    task_id=task_id,
                    error_code=err_code,
                    sensors_config_id=sensor_config_id,
                    apollo_config_id=apollo_config_id,
                    scenario_id=scenario_config_id,
                    simulator_version=simulator_version)
            if not ok:
                err_code = ErrorCodes.DB_ERROR_FAIL_TO_WRITE
            return err_code

        # using default configs
        fps = 50
        log_dir='./log'
        ego_role_name='hero'
        carla_timeout=20.0
        show=False

        # TODO
        # duration is hardcoded to 5 mins because there is
        # no way to tell how long a simulation will run
        # for now.
        apollo_host, carla_host = acquire_servers(
            stop_event, centre_endpoint, duration=60*5)
        logging.info(f"acquired apollo server: {apollo_host}; "
                    f"carla server :{carla_host}")
        carla_host = carla_host
        carla_port = 2000
        apollo_host = apollo_host
        apollo_port = 9090
        dreamview_port = 8888

    except Exception as e:
        logging.error(f"unknown error in acquiring servers, {e}")
        if not stop_event.is_set():
            stop_event.set()
        err_code = ErrorCodes.UNKNOWN_ERROR
        ok = write_to_db(
                db_user=db_user,
                db_password=db_password,
                db_host=db_host,
                db_port=db_port,
                database=database,
                task_id=task_id,
                error_code=err_code,
                sensors_config_id=sensor_config_id,
                apollo_config_id=apollo_config_id,
                scenario_id=scenario_config_id,
                simulator_version=simulator_version)
        if not ok:
            err_code = ErrorCodes.DB_ERROR_FAIL_TO_WRITE
        return err_code

    ############################################################
    # start simulation
    ############################################################
    start_time = create_timestamp()
    result = start_simulation(
        stop_event=stop_event,
        apollo_host=apollo_host,
        apollo_port=apollo_port,
        dreamview_port=dreamview_port,
        carla_host=carla_host,
        carla_port=carla_port,
        scenario_name=scenario_name,
        scenario_config_tree=xml_tree,
        sensor_config=sensor_config,
        apollo_config=apollo_config,
        fps=fps,
        log_dir=log_dir,
        ego_role_name=ego_role_name,
        carla_timeout=carla_timeout,
        show=show)
    if not stop_event.is_set():
        stop_event.set()

    end_time = create_timestamp()
    err_code = result.err_code
    criteria = result.criteria
    ok = write_to_db(
            db_user=db_user,
            db_password=db_password,
            db_host=db_host,
            db_port=db_port,
            database=database,
            task_id=task_id,
            error_code=err_code,
            sensors_config_id=sensor_config_id,
            apollo_config_id=apollo_config_id,
            scenario_id=scenario_config_id,
            simulator_version=simulator_version,
            carla_ip=carla_host,
            carla_port=carla_port,
            apollo_ip=apollo_host,
            dreamview_port=dreamview_port,
            bridge_port=apollo_port,
            start_time=start_time,
            end_time=end_time,
            criteria=criteria)
    if not ok:
        err_code = ErrorCodes.DB_ERROR_FAIL_TO_WRITE
    return err_code
