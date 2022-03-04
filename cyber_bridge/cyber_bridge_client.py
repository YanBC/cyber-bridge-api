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
from typing import List
from google.protobuf.descriptor_pb2 import FileDescriptorProto
from google.protobuf.message import Message
import logging


####################################
# cyber_bridge ops
####################################
class OP(enum.Enum):
    OP_REGISTER_DESC = 1
    OP_ADD_READER = 2
    OP_ADD_WRITER = 3
    OP_PUBLISH = 4


def op_register(file_desc) -> List[bytes]:
    all_deps = []

    def find_all_deps(_file_desc):
        nonlocal all_deps
        for dep in _file_desc.dependencies:
            find_all_deps(dep)
        all_deps.append(_file_desc)
    find_all_deps(file_desc)

    opType = OP.OP_REGISTER_DESC
    count = 1
    desc_strs = []
    # all_deps = find_all_deps(file_desc)
    for dep in all_deps:
        desc_str = b''
        proto = FileDescriptorProto()
        dep.CopyToProto(proto)
        proto.name = dep.name
        dataBytes = proto.SerializeToString()
        dataLen = len(dataBytes)

        desc_str += opType.value.to_bytes(1, byteorder='little')
        desc_str += count.to_bytes(4, byteorder='little')
        desc_str += dataLen.to_bytes(4, byteorder='little')
        desc_str += dataBytes
        desc_strs.append(desc_str)

    return desc_strs


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
# Messenger
####################################
class BufferedSocket:
    def __init__(self, host:str, port:int) -> None:
        self._socket = None
        self.remote_host = host
        self.remote_port = port
        self._recv_carry_bytes = b""

    def connect(self) -> bool:
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self._socket.settimeout(0.05)
        self._socket.settimeout(0.1)
        self._socket.connect((self.remote_host, self.remote_port))
        return True

    def send(self, msgBytes:List[bytes]) -> bool:
        for msg in msgBytes:
            self._socket.sendall(msg)
        return True

    def recv(self, msgLength:int=2**10) -> dict:
        '''
        raise ValueError
        '''
        ret = dict()
        dataBytes = b''

        try:
            dataBytes = self._socket.recv(msgLength)
        except socket.timeout:
            # logging.warning("socket receive timed out")
            pass

        if len(dataBytes) == 0:
            return ret
        dataBytes = self._recv_carry_bytes + dataBytes
        offset = 0

        dataLen = len(dataBytes)
        while dataLen - offset > 1:
            if dataBytes[offset] != 0x04:
                raise ValueError
            else:
                tmp_offset = offset + 1

                # channel name size
                if dataLen - tmp_offset < 4:
                    break
                sizeBytes = dataBytes[tmp_offset:tmp_offset+4]
                tmp_offset += 4
                chanSize = int.from_bytes(sizeBytes, byteorder='little')

                # channel name
                if dataLen - tmp_offset < chanSize:
                    break
                chanBytes = dataBytes[tmp_offset:tmp_offset+chanSize]
                tmp_offset += chanSize

                # message size
                if dataLen - tmp_offset < 4:
                    break
                sizeBytes = dataBytes[tmp_offset:tmp_offset+4]
                tmp_offset += 4
                msgSize = int.from_bytes(sizeBytes, byteorder='little')

                # message
                if dataLen - tmp_offset < msgSize:
                    break
                msgBytes = dataBytes[tmp_offset:tmp_offset+msgSize]
                tmp_offset += msgSize

                channel_name = chanBytes.decode()
                ret[channel_name] = msgBytes
                offset = tmp_offset

        self._recv_carry_bytes = dataBytes[offset:]
        return ret


class CyberBridgeClient:
    def __init__(
                self, host:str, port:int,
                encoders:list, decoders:list) -> None:
        self.socket = BufferedSocket(host, port)
        self.encoders = encoders
        self.decoders = decoders
        self.msg_map = dict()
        for decoder in self.decoders:
            self.msg_map[decoder.channel] = decoder.pbCls

    def initialize(self) -> bool:
        msg_list = []
        for e in self.encoders:
            msg_list += e.get_register_bytes()
        msg_list += [e.get_add_writer_bytes() for e in self.encoders]
        msg_list += [d.get_add_reader_bytes() for d in self.decoders]

        if self.socket.connect():
            return self.socket.send(msg_list)
        else:
            return False

    def send_pb_messages(self, pb_msgs:List[bytes]) -> bool:
        return self.socket.send(pb_msgs)

    def recv_pb_messages(self) -> List[Message]:
        msg_dict = self.socket.recv()
        ret = []
        for channel_name in msg_dict.keys():
            if channel_name in self.msg_map:
                msgBytes = msg_dict[channel_name]
                pbCls = self.msg_map[channel_name]()
                pbCls.ParseFromString(msgBytes)
                ret.append(pbCls)
        return ret


if __name__ == '__main__':
    def dummy_source(cyber_bridge_sock, freq=10):
        import random
        import time
        from google.protobuf.descriptor_pb2 import FileDescriptorProto
        from modules.perception.proto.traffic_light_detection_pb2 import TrafficLightDetection

        channel = "/apollo/perception/traffic_light"
        dataType = "apollo.perception.TrafficLightDetection"

        registerBytes_list = op_register(TrafficLightDetection.DESCRIPTOR.file)
        addWriterBytes = op_add_writer(channel, dataType)

        # sendBytes = registerBytes + addWriterBytes
        sendBytes_list = registerBytes_list + [addWriterBytes]
        sleepTime = 1 / freq

        for sendBytes in sendBytes_list:
            cyber_bridge_sock.send([sendBytes])

        # time.sleep(1000000)
        while True:
            time.sleep(sleepTime)
            msgBytes = b""
            for i in range(100):
                tmp = random.randint(0, 255)
                msgBytes += tmp.to_bytes(1, byteorder='little')
            publishBytes = op_publish(channel, msgBytes)
            print(f"### Send: {publishBytes}")
            cyber_bridge_sock.send([publishBytes])

    def dummy_sink(cyber_bridge_sock):
        from modules.control.proto.control_cmd_pb2 import ControlCommand
        channel = "/apollo/control"
        dataType = "apollo.control.ControlCommand"

        registerBytes_list = []
        # registerBytes_list += op_register(ControlCommand.DESCRIPTOR.file)
        addReaderBytes = op_add_reader(channel, dataType)

        sendBytes_list = registerBytes_list + [addReaderBytes]
        cyber_bridge_sock.send(sendBytes_list)
        while True:
            data = cyber_bridge_sock.recv(65536)
            print(f"### Receive: {data}")

    host = '172.17.0.3'
    port = 9090
    cb_sock = BufferedSocket(host, port)
    cb_sock.connect()
    # dummy_sink(cb_sock)
    dummy_source(cb_sock, freq=1)
