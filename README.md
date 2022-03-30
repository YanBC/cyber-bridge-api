# Overview
File tree:
```bash
.
├── compile_proto.sh            # script for compiling proto files
├── cyber                       # proto files from Apollo Autopilot
├── cyber_bridge                # codes for communicating with Apollo cyber_bridge
├── Dockerfile                  # Dockerfile
├── dreamview_api.py            # Apollo Dreamview websocket api
├── main.py                     # main script
├── modules                     # proto files from Apollo Autopilot
├── opendrive                   # opendrive maps
├── pygame_viewer.py            # pygame viewer sub-process
├── README.md                   # this README.md
├── requirements.txt            # required python packages
├── scenario_configs            # scenario configurations, unused for now
├── scenario_runner             # scenario_runner submodule
├── sensor_configs              # sensors configurations
├── sensors                     # codes for sensors
├── setup.bash                  # setup environment variables
└── utils.py                    # common utilities codes
```

## How to run
```bash
# 0. good programmers start counting at zero
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
python main.py --carla-host <carla.server.ip.addr> --apollo-host <apollo.ip.addr> --configFile scenario_configs/781_stop_at_fix_location_cfg.json
```

## Components
A functioning simulator node should contains the following components:
- Carla Sensors Module
- Control Sensor Module
- Pygame Viewer
- Scenario Runner
- Apollo Bootstrap

## Available Sensors
1. `ChassisAlter`
2. `BestPose`
3. `Odometry`
4. `InsStatus`
5. `CorrectedImu`
6. `Obstacles`
7. `TrafficLightAlter`
8. `ClockSensor`

Common attributes for all sensors:
- "name": string, id of the sensor, used for identification, should be different for different sensor instances
- "type": string, type of the sensor class
- "frequency": number, set it to -1.0 if the sensor should be run as fast as it can

### ChassisAlter
This sensor sends data about the vehicle chassis.

### BestPose
This sensor outputs the GPS location of the vehicle in Longitude/Latitude and Northing/Easting coordintates.

### Odometry
This sensor outputs the GPS location of the vehicle in Longitude/Latitude and Northing/Easting coordintates and the vehicle velocity.

Since some maps do not centred at (0,0), there are two additional attributes for this sensor:
- "x_offset": number, offset along the x axis
- "y_offset": number, offset along the y axis


### InsStatus
This sensor outputs the status of the GPS correction due to INS. The simulator is an ideal environment in which GPS is always corrected.

### CorrectedImu
This sensor output the data about the vehicle imu status.


### Obstacles
This sensor returns 3D ground truth data for training and creates bounding boxes around the detected objects.

Additional attributes:
- "x_offset": number, offset along the x axis
- "y_offset": number, offset along the y axis
- "vehicle_distance": number, vehicles outside of this radius will not be detected
- "walker_distance": number, pedestrians outside of this radius will not be detected


### TrafficLightAlter
This sensor returns ground truth data for traffic light signals connected to the current lane of ego vehicle.

### Clock
This sensor outputs simulated time to CyberRT as clock messages.



## Using Docker
Build docker image
```bash
git clone --recurse-submodules git@gitlab.quyan.info:the-flying-hoyshanist/simulator.git
cd ./simulator
docker build -t simulator-0.9.11:v0.1 .
```

Start docker container
```bash
docker run -it -d \
-v /tmp/.X11-unix/:/tmp/.X11-unix \
simulator-0.9.11:v0.1 \
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



### Compile protos
```bash
bash compile_proto.sh
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
