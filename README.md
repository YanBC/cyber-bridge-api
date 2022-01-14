# Overview
File tree:
```bash
.
├── main.py                     # main script
├── compile_proto.sh            # script for compiling proto files
├── cyber                       # proto files from Apollo Autopilot
├── cyber_bridge                # codes for communicating with Apollo cyber_bridge
├── Dockerfile                  # Dockerfile
├── modules                     # proto files from Apollo Autopilot
├── opendrive                   # opendrive maps
├── pygame_viewer.py            # pygame viewer sub-process
├── README.md                   # this README.md
├── requirements.txt            # required python packages
├── scenario_configs            # scenario configurations, unused for now
├── scenario_runner.py          # scenario_runner.py from scenario runner project
├── sensor_configs              # sensors configurations
├── sensors                     # codes for sensors
└── utils.py                    # common utilities codes
```


## TO-DO
1. include scenario_runner as a git submodule, and start scenerio_manager as a child process
2. turn all `print`s into `log`s
3. <del>project docker image (Dockerfile)</del>
4. apollo opendrive map converter (*.xodr -> apollo_map)
5. communication between scenario_runner and main process
6. <del>different frequency for different sensors</del>
7. <del>sensor configuration files</del>


## Components
A functioning simulator node should contains the following components:
- Carla Sensors Module
- Control Sensor Module
- Pygame Viewer
- Scenario Runner
- Apollo Bootstrap

### Carla Sensors Module
Algorithm:
1. Wait for ego vehicle to be created
2. Setup sensors on ego
3. For every server tick, update sensors and send msg to Apollo
4. When done, destroy all created sensors (Carla actors)

Exit condition:
Process exits when ego no longer exists

### Control Sensor Module
Algorithm:
1. Wait for ego vehicle to be created
2. For every server tick, receive apollo control commands and apply to ego

Exit condition:
Process exits when ego no longer exists

### Pygame Viewer
Algorithm:
1. Wait for ego vehicle to be created
2. Create an rgb camera sensor attached to ego, and pipe output dataframe to pygame
3. For every server tick, update ego status and pygame windows
4. When done, destroy all created sensors (Carla actors)

Exit condition:
Process exits when one of the following happens:
1. Ego vehicle no longer exists
2. User prompted exit (via pygame interface)

### Scenario Runner (TODO)
Algorithm:
1. Setup Carla world (map, weather, etc)
2. Create ego vehicle and other actors (vehicles and walkers)
3. For every server tick, monitor ego vehicle metrics and manipulate other actors' behavour accordingly
4. Output scenario report
5. Destroy all carla actors created in step 2

Exit condition:
Process exits when one of the following happens:
1. Timeout
2. Ego vehicle violate one of the criteria


### Apollo Bootstrap (TODO)
Algorithm:
1. Setup Apollo map
2. Setup Apollo vehicle
3. Load all required Apollo modules
4. Send Apollo routing request
5. Monitor Apollo moduels status
6. Output abnormalies if any

Exit condition:
Process exits when indicated (probably a `multiprocessing.Event`?)


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
docker build -t cyber-bridge-api:0.9.11 .
```

Start docker container
```bash
docker run -it -d \
-v /tmp/.X11-unix/:/tmp/.X11-unix \
cyber-bridge-api:0.9.11 \
bash
```


## Misc
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

### env setting
export SCENARIO_RUNNER_ROOT="/third-parties/scenario_runner/"

