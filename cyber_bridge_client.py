"""
Apollo cyber_bridge client

Operations:
1. Write:
    - register
    - add_writer
    - publish
2. Read:
    - add_reader
"""

import socket
import enum
import importlib
import time
from google.protobuf.descriptor_pb2 import FileDescriptorProto


####################################
# cyber_bridge ops
####################################
class OP(enum.Enum):
    OP_REGISTER_DESC = 1
    OP_ADD_READER = 2
    OP_ADD_WRITER = 3
    OP_PUBLISH = 4


def op_register(file_desc) -> bytes:

    def get_desc_str(file_desc, count):
        """
        register proto message desc file.
        """
        ret = b''
        for dep in file_desc.dependencies:
            ret, count = get_desc_str(dep, count)
        proto = FileDescriptorProto()
        file_desc.CopyToProto(proto)
        proto.name = file_desc.name
        dataBytes = proto.SerializeToString()
        dataLen = len(dataBytes)
        ret += dataLen.to_bytes(4, byteorder='little')
        ret += dataBytes
        count += 1
        return ret, count

    count = 0
    desc_str, count = get_desc_str(file_desc, count)
    opType = OP.OP_REGISTER_DESC

    ret = b""
    ret += opType.value.to_bytes(1, byteorder='little')
    ret += count.to_bytes(4, byteorder='little')
    ret += desc_str

    return ret


def op_add_reader(chan: str, data: str) -> bytes:
    opType = OP.OP_ADD_READER
    chanBytes = chan.encode()
    chanLen = len(chanBytes)
    dataBytes = data.encode()
    dataLen = len(dataBytes)

    ret = b""
    ret += opType.value.to_bytes(1, byteorder='little')
    ret += chanLen.to_bytes(4, byteorder='little')
    ret += chanBytes
    ret += dataLen.to_bytes(4, byteorder='little')
    ret += dataBytes

    return ret


def op_add_writer(chan: str, data: str) -> bytes:
    opType = OP.OP_ADD_WRITER
    chanBytes = chan.encode()
    chanLen = len(chanBytes)
    dataBytes = data.encode()
    dataLen = len(dataBytes)

    ret = b""
    ret += opType.value.to_bytes(1, byteorder='little')
    ret += chanLen.to_bytes(4, byteorder='little')
    ret += chanBytes
    ret += dataLen.to_bytes(4, byteorder='little')
    ret += dataBytes

    return ret


def op_publish(chan: str, message: bytes) -> bytes:
    opType = OP.OP_PUBLISH
    chanBytes = chan.encode()
    chanLen = len(chanBytes)
    msgBytes = message
    msgLen = len(msgBytes)

    ret = b""
    ret += opType.value.to_bytes(1, byteorder='little')
    ret += chanLen.to_bytes(4, byteorder='little')
    ret += chanBytes
    ret += msgLen.to_bytes(4, byteorder='little')
    ret += msgBytes

    return ret


####################################
# dummy source and sinks
####################################
def dummy_source(freq=10):
    import random
    import time
    from google.protobuf.descriptor_pb2 import FileDescriptorProto
    from modules.perception.proto.traffic_light_detection_pb2 import TrafficLightDetection

    host = '127.0.0.1'
    port = 9090
    channel = "/apollo/perception/traffic_light"
    dataType = "apollo.perception.TrafficLightDetection"

    registerBytes = op_register(TrafficLightDetection.DESCRIPTOR.file)
    addWriterBytes = op_add_writer(channel, dataType)
    
    sleepTime = 1 / freq
    sendBytes = registerBytes + addWriterBytes
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((host, port))
        s.sendall(sendBytes)

        # time.sleep(1000000)
        while True:
            time.sleep(sleepTime)
            msgBytes = b""
            for i in range(100):
                tmp = random.randint(0, 255)
                msgBytes += tmp.to_bytes(1, byteorder='little')
            publishBytes = op_publish(channel, msgBytes)
            print(f"### Send: {publishBytes}")
            s.sendall(publishBytes)


def dummy_sink():
    host = '127.0.0.1'
    port = 9090
    channel = "/apollo/control"
    dataType = "apollo.control.ControlCommand"

    sendBytes = op_add_reader(channel, dataType)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((host, port))
        s.sendall(sendBytes)

        while True:
            data = s.recv(65536)
            print(f"### Receive: {data}")



if __name__ == '__main__':
    # dummy_sink()
    dummy_source(freq=1)

