## Compile protos
```bash
bash compile_proto.sh
```


## TO-DO
1. include scenario_runner as a git submodule, and start scenerio_manager as a child process
2. turn all `print`s into `log`s
3. <del>project docker image (Dockerfile)</del>
4. apollo opendrive map converter (*.xodr -> apollo_map)
5. communication between scenario_runner and main process


## About Apollo Map
- Apollo Autopilot maps use a modified version of opendrive, so directly export carla maps to opendrive format won't be compatible with Apollo.
- AuroAI has converted all carla maps to Apollo maps format, [see here](https://auro.ai/blog/2020/03/using-open-source-frameworks-in-autonomous-vehicle-development-part-2/)


## About lgsvl Maps
https://github.com/lgsvl/simulator/issues/1416


## Disable carla server rendering for higher fps
```bash
# in carla/PythonAPI/util
python3 config.py --no-rendering
```

See https://carla.readthedocs.io/en/0.9.11/adv_rendering_options/ for details.


## Build docker image
```bash
docker build -t cyber-bridge-api:0.9.11 .
```

## Start docker container
```bash
docker run -it -d \
-v /tmp/.X11-unix/:/tmp/.X11-unix \
cyber-bridge-api:0.9.11 \
bash
```

## Start Scenario
```bash
# clone this project inside your docker container
git clone https://github.com/YanBC/cyber-bridge-api.git && \
cd cyber-bridge-api

# run NoSignalJunctionCrossing scenario
python scenario_runner.py --host <caral_server_ip> --scenario NoSignalJunctionCrossing --output
# or if you prefer, run FreeRide_3 scenario
# python scenario_runner.py --host <caral_server_ip> --scenario FreeRide_3 --output

# run simulator client
# note that in order to see any graphics, you have to set the DISPLAY env variable
python carla_run_vehicle.py --carla <caral_server_ip> --apollo <apollo_container_ip> --show
```
