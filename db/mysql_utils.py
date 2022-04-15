import mysql.connector
from mysql.connector import errorcode
from db.db_sqls import (
    create_table_results_sql,
    create_table_criteria_sql,
    create_table_error_codes_sql,
    insert_results_sql,
    insert_criteria_sql,
    insert_error_codes_sql,)
from db.error_codes import err_table


def create_connection(
        user: str,
        password: str,
        host: str,
        database: str,
        port: int = 3306):
    try:
        cnx = mysql.connector.connect(
                user=user, password=password, host=host, database=database)
    except mysql.connector.Error as err:
        if err.errno == errorcode.ER_ACCESS_DENIED_ERROR:
            raise RuntimeError("mysql: user or password is wrong")
        elif err.errno == errorcode.ER_BAD_DB_ERROR:
            raise RuntimeError("mysql: database doesn't exist")
        else:
            raise err
    else:
        return cnx


# write to table results
def _insert_reulsts(
                cnx,
                task_id,
                simulator_version,
                carla_version,
                carla_ip,
                carla_port,
                apollo_version,
                apollo_ip,
                dreamview_port,
                bridge_port,
                error_code,
                sensors_config_id,
                start_time,
                end_time,
                ego,
                scenario_id,
                apollo_config_id):
    cursor = cnx.cursor()
    params = {
        "task_id": task_id,
        "simulator_version": simulator_version,
        "carla_version": carla_version,
        "carla_ip": carla_ip,
        "carla_port": carla_port,
        "apollo_version": apollo_version,
        "apollo_ip": apollo_ip,
        "dreamview_port": dreamview_port,
        "bridge_port": bridge_port,
        "error_code": error_code,
        "sensors_config_id": sensors_config_id,
        "start_time": start_time,
        "end_time": end_time,
        "ego": ego,
        "scenario_id": scenario_id,
        "apollo_config_id": apollo_config_id
    }
    try:
        cursor.execute(insert_results_sql, params)
    finally:
        cursor.close()

# write to table criteria
def _insert_criteria(
                    cnx,
                    task_id,
                    c_type,
                    expected,
                    actual,
                    success):
    cursor = cnx.cursor()
    params = {
        "task_id": task_id,
        "c_type": c_type,
        "expected": expected,
        "actual": actual,
        "success": success,
    }
    try:
        cursor.execute(insert_criteria_sql, params)
    finally:
        cursor.close()

# save simulation results
def save_result(
            cnx,
            task_id,
            error_code,
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
            criteria: list):
    try:
        _insert_reulsts(cnx, task_id, simulator_version, carla_version, carla_ip, carla_port, apollo_version, apollo_ip, dreamview_port, bridge_port, error_code, sensors_config_id, start_time, end_time, ego, scenario_id, apollo_config_id)
        for c in criteria:
            c_type = c['name']
            expected = str(c['expected'])
            actual = str(c['actual'])
            success = str(c['success'])
            _insert_criteria(cnx, task_id, c_type, expected, actual, success)
    except Exception as e:
        cnx.rollback()
        raise e


# write to table error codes
def insert_error_codes(cnx, code, message):
    cursor = cnx.cursor()
    params = {
        "code": code,
        "message": message
    }
    try:
        cursor.execute(insert_error_codes_sql, params)
    finally:
        cursor.close()


# create tables
def _create_tables(cnx):
    cursor = cnx.cursor()
    try:
        cursor.execute(create_table_results_sql)
        cursor.execute(create_table_criteria_sql)
        cursor.execute(create_table_error_codes_sql)
    finally:
        cursor.close()


# initial_database
def initialize_database(cnx):
    try:
        _create_tables(cnx)
        for c in err_table:
            insert_error_codes(cnx, c.value, err_table[c])
    except Exception as e:
        cnx.rollback()
        raise e
