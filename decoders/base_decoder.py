from cyber_bridge_client import (
    op_add_reader
)


class BaseDecoder:
    def __init__(self, pbCls, channel:str, msgType:str) -> None:
        self.pbCls = pbCls
        self.channel = channel
        self.msgType = msgType

    def get_add_reader_bytes(self) -> bytes:
        return op_add_reader(self.channel, self.msgType)
