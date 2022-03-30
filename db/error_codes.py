'''
error codes and messages mapping
'''
import enum


class ErrorCodes(enum.Enum):
    SUCCESS = 0
    CONFIG_ERROR = 1000
    NETWORK_ERROR  = 2000
    NETWORK_ERROR_CARLA = 2001
    NETWORK_ERROR_APOLLO = 2002
    NETWORK_ERROR_REDIS = 2004
    APOLLO_BOOSTRAP_ERROR = 3000
    SIMULATION_FAIL = 4000
    OTHER_ERROR = 5000
    USER_INTERRUPT = 5001
    UNKNOWN_ERROR = 9999


err_table = {
    ErrorCodes.SUCCESS: "成功",
    ErrorCodes.CONFIG_ERROR: "配置错误",
    ErrorCodes.NETWORK_ERROR: "网络连接错误",
    ErrorCodes.NETWORK_ERROR_CARLA: "Carla服务器连接失败",
    ErrorCodes.NETWORK_ERROR_APOLLO: "Apollo连接失败",
    ErrorCodes.NETWORK_ERROR_REDIS: "Redis连接失败",
    ErrorCodes.APOLLO_BOOSTRAP_ERROR: "Apollo bootstrap错误",
    ErrorCodes.SIMULATION_FAIL: "仿真场景测试失败",
    ErrorCodes.OTHER_ERROR: "其他错误",
    ErrorCodes.USER_INTERRUPT: "用户中断",
    ErrorCodes.UNKNOWN_ERROR: "未知错误"
}
