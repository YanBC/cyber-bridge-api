# Database

配置类信息放到配置中心，暂时用nacos做配置中心，总共两张表：
1. `results`: 仿真测试任务结果表，每一列对应一次仿真任务
2. `criteria`：仿真测试指标结果表，每一列对应一次仿真任务的一个测试指标

## Table `results`
|    COLUMNS   | TYPE | NOTES |
| ----------- | ----------- | ----------- |
| task_id| numeric| 测试id，主键|
| simulator_version| string| simulator版本|
| carla_version | string | carla 服务器版本 | 
| carla_ip | string | carla服务器ip地址 |
| carla_port| string| caral服务器端口|
| apollo_version| string| apollo 服务器版本|
| apollo_ip| string|  apollo服务器ip|
| apollo_port| string| caral服务器端口|
| error_code| numeric| 错误码|
| sensors_config_id| string| 可以根据sensors_config_id，通过nacos获取完整的传感器配置信息，大概类似`sensor_configs/apollo_600_modular_testing.json`|
| start_time | timestamp| 测试开始时间|
| end_time| timestamp| 测试结束时间|
| ego| string| ego车辆型号|
| scenario_id| string| 可以根据scenario_id，通过nacos获取完整的传感器配置信息，大概类似`scenario_configs/711_speed_limit_cfg.xml`外加测试指标|
| created_time| timestamp | 数据写入时间|

## Table `criteria`
|    COLUMNS   | TYPE | NOTES |
| ----------- | ----------- | ----------- |
| c_id| numeric| 主键，自然递增|
| task_id| numeric | 外键，测试id|
| c_type| string| 测试指标类型名称|
| expected| string| 预期结果，根据测试指标类型解析|
| actual| string| 实际结果，根据测试指标类型解析|
| success| numeric| 是否通过，0是通过成功，1是通过失败|
| created_time| timestamp | 数据写入时间|

## Table `error_codes` (read-only)
|    COLUMNS   | TYPE | NOTES |
| ----------- | ----------- | ----------- |
| e_id | numeric| 主键，自然递增|
| code | numeric | 错误码 |
| message| string | 错误信息|
| created_time| timestamp | 创建时间|
| updated_time| timestamp | 修改时间|


## Current error codes
- `0`: 成功
- `1XXX`: 配置错误
- `2XXX`: 网络连接失败
    - `2001`: Carla服务器连接失败
    - `2002`: Apollo连接失败
    - `2003`: nacos连接失败
    - `2004`: redis连接失败
- `3XXX`: Apollo bootstrap失败
- `4XXX`: 仿真场景测试失败
- `5XXX`: 其他错误
