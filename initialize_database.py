import argparse
from db.mysql_utils import (
    create_connection,
    initialize_database
)

def get_args():
    p = argparse.ArgumentParser()
    p.add_argument("host", help="mysql host ip")
    p.add_argument("--port", default="3306", help="mysql port")
    p.add_argument("db", help="mysql database name")
    p.add_argument("user", help="mysql username")
    p.add_argument("password", help="user password")
    return p.parse_args()


if __name__ == '__main__':
    args = get_args()
    host = args.host
    port = args.port
    database = args.db
    user = args.user
    password = args.password

    cnx = create_connection(user, password, host, database, port=port)
    try:
        initialize_database(cnx)
        cnx.commit()
    finally:
        cnx.close()
