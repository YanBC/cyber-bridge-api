# Overview

项目主要文件如下：
```bash
.
├── apollo_configs                  # Apollo 配置文件
├── compile_proto.sh                # proto文件生成脚本
├── cyber                           # proto files from Apollo Autopilot
├── cyber_bridge                    # Apollo bridge
├── db                              # 数据库相关代码
├── dreamview_api.py                # Apollo dreamview交互代码
├── grpc_simaster                   # grpc 服务代码
├── main.py                         # standalone 仿真器运行脚本
├── modules                         # proto files from Apollo Autopilot
├── pygame_viewer.py                # 仿真 gui 可视化代码
├── README.md                       # this README.md
├── scenario_configs                # scenario 配置文件
├── sensor_configs                  # sensor 配置文件
├── sensors                         # 传感器代码
├── service_discovery               # nacos服务发现和配置中心交互代码
├── setup.bash                      # python环境配置脚本
├── simaster_client_example.py      # grpc 客户端示例代码
├── simaster_config.json            # grpc 服务器示例配置
├── simulation.py                   # main simulation codes
├── start_simaster.py               # grpc 服务启动脚本
├── utils.py                        # common utilities codes
└── VERSION                         # 版本信息
```


部分`README`及其内容：
|   location   |  about |
| ----------- | ----------- |
| `db/README.md` |数据库设计 |
| `grpc_simaster/README.md` | grpc 服务 |
| `sensors/README.md` | 目前支持的传感器 |



## How to run
```bash
# 0. download repo
git clone --recurse-submodules git@gitlab.quyan.info:the-flying-hoyshanist/simulator.git
# 1. cd to project root
cd simulator/
# 2. generate protobuf files
bash compile_proto.sh
# 3. source setup script
source setup.bash
# 4. show usage
python main.py -h
# 5. run main.py script
python main.py --carla-host <carla.server.ip.addr> --apollo-host <apollo.ip.addr>
```



## Using Docker
Build docker image
```bash
git clone --recurse-submodules git@gitlab.quyan.info:the-flying-hoyshanist/simulator.git
cd ./simulator
docker build -t simulator-0.9.11 .
```

Start docker container
```bash
docker run -it -d \
-v /tmp/.X11-unix/:/tmp/.X11-unix \
simulator-0.9.11 \
bash
```


## Misc

### Setup python environment
```bash
docker pull carlasim/carla:0.9.11 && \
docker run -it -d --rm --name carla_temp carlasim/carla:0.9.11 bash && \
docker cp carla_temp:/home/carla/PythonAPI/carla/dist/carla-0.9.11-py3.7-linux-x86_64.egg . && \
docker stop carla_temp

python3.7 -m venv pyVir
source pyVir/bin/activate
python -m easy_install carla-0.9.11-py3.7-linux-x86_64.egg && rm carla-0.9.11-py3.7-linux-x86_64.egg
python -m pip install -r requirements.txt
```


### About Apollo Map
- Apollo Autopilot maps use a modified version of opendrive, so directly export carla maps to opendrive format won't be compatible with Apollo.
- AuroAI has converted all carla maps to Apollo maps format, [see here](https://auro.ai/blog/2020/03/using-open-source-frameworks-in-autonomous-vehicle-development-part-2/)


### About lgsvl Maps
https://github.com/lgsvl/simulator/issues/1416


### Disable carla server rendering for higher fps
```bash
# in carla/PythonAPI/util
python3 config.py --no-rendering
```

See https://carla.readthedocs.io/en/0.9.11/adv_rendering_options/ for details.
