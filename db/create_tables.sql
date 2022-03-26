CREATE TABLE IF NOT EXISTS results (
    task_id BIGINT NOT NULL COMMENT '任务id',
    simulator_version VARCHAR (64) NOT NULL COMMENT 'simulator版本',
    carla_version VARCHAR (64) NOT NULL COMMENT 'carla 服务器版本',
    carla_ip VARCHAR (64) NOT NULL COMMENT 'carla服务器ip地址',
    carla_port VARCHAR (64) NOT NULL COMMENT 'carla服务器端口',
    apollo_version VARCHAR (64) NOT NULL COMMENT 'apollo服务器版本',
    apollo_ip VARCHAR (64) NOT NULL COMMENT 'apollo服务器ip地址',
    apollo_port VARCHAR (64) NOT NULL COMMENT 'apollo服务器端口',
    error_code INT NOT NULL COMMENT '错误码',
    sensors_config_id VARCHAR (128) NOT NULL COMMENT '传感器配置id',
    start_time TIMESTAMP NOT NULL COMMENT '仿真测试开始时间',
    end_time TIMESTAMP NOT NULL COMMENT '仿真测试结束时间',
    ego VARCHAR (64) NOT NULL COMMENT 'ego车辆型号',
    scenario_id VARCHAR (128) NOT NULL COMMENT 'nacos场景配置id',
    created_time TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP COMMENT '数据写入时间',
    PRIMARY KEY (task_id)
) ENGINE = InnoDB DEFAULT CHARSET = utf8mb4 COMMENT = '仿真测试任务结果表';

CREATE TABLE IF NOT EXISTS criteria (
    c_id BIGINT NOT NULL AUTO_INCREMENT,
    task_id BIGINT NOT NULL COMMENT '任务id',
    c_type VARCHAR (64) NOT NULL COMMENT '测试指标类型名称',
    expected VARCHAR (128) NOT NULL COMMENT '预期结果，根据测试指标类型解析',
    actual VARCHAR (128) NOT NULL COMMENT '实际结果，根据测试指标类型解析',
    success INT NOT NULL COMMENT '指标是否通过, 0是通过, 1是不通过',
    created_time TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP COMMENT '数据写入时间',
    PRIMARY KEY (c_id),
    FOREIGN KEY (task_id) REFERENCES results (task_id)
) ENGINE = InnoDB DEFAULT CHARSET = utf8mb4 COMMENT = '仿真测试指标结果表';

CREATE TABLE IF NOT EXISTS error_codes (
    e_id BIGINT NOT NULL AUTO_INCREMENT,
    code INT NOT NULL COMMENT '错误码',
    message VARCHAR (4096) not NULL comment '报错消息',
    created_time TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP COMMENT '创建时间',
    updated_time TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP ON UPDATE CURRENT_TIMESTAMP COMMENT '修改时间',
    primary key (e_id)
) ENGINE = InnoDB DEFAULT CHARSET = utf8mb4 COMMENT = '仿真测试错误码';