#!/usr/bin/env python3
import http.client
import logging
import string
import json
import time


class Client():
    def __init__(self, ip='', port=8080, timeout=10) -> None:
        self._ip = ip
        self._port = port
        self._timeout = timeout
        self.connection = http.client.HTTPConnection(
            self._ip, self._port, self._timeout)
        logging.basicConfig(level=logging.INFO)

    def do_get(self):
        self.connection.request("GET", "/")
        response = self.connection.getresponse()
        logging.info("Status: {} and reason: {}".format(
            response.status, response.reason))

    def do_pose(self, data: string):
        headers = {'Content-type': 'application/json'}
        json_data = json.dumps(data)
        self.connection.request('POST', '/post', json_data, headers)
        response = self.connection.getresponse()
        logging.info("Status: {} and reason: {}".format(
            response.status, response.reason))

    def do_put(self, name: string, data: string):
        headers = {'Content-type': 'application/json'}
        json_data = json.dumps(data)
        self.connection.request("PUT", "/{}".format(name), json_data, headers)
        response = self.connection.getresponse()
        logging.info("Status: {} and reason: {}".format(
            response.status, response.reason))

    def close(self):
        if self.connection:
            self.connection.close()


if __name__ == '__main__':
    client = Client(ip='172.17.0.27')
    # client.do_get()
    # client.do_pose("{'text': 'Hello HTTP #1 **cool**, and #1!'}")
    client.do_put("{'text': 'Hello HTTP #1 **cool**, and #1!'}")
    time.sleep(1)
    client.close()
