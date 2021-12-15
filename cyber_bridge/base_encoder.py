from google.protobuf.message import Message
from typing import List
from cyber_bridge.cyber_bridge_client import (
    op_register,
    op_add_writer,
    op_publish
)


class BaseEncoder:
    def __init__(self, pbCls, channel:str, msgType:str) -> None:
        self.pbCls = pbCls
        self.channel = channel
        self.msgType = msgType

    def get_register_bytes(self) -> List[bytes]:
        return op_register(self.pbCls.DESCRIPTOR.file)

    def get_add_writer_bytes(self) -> bytes:
        return op_add_writer(self.channel, self.msgType)

    def get_publish_bytes(self, pbInstance:Message) -> bytes:
        '''
        raise ValueError
        '''
        if not isinstance(pbInstance, self.pbCls):
            raise ValueError
        msgBytes = pbInstance.SerializeToString()
        return op_publish(self.channel, msgBytes)
