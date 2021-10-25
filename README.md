## Compile protos
```bash
bash compile_proto.sh
```

## Run
```bash
env \
PYTHONPATH=/data/workspace/home/yanbc/workspace/codes/carla/PythonAPI/carla:/data/workspace/home/yanbc/workspace/codes/carla/PythonAPI:/data/workspace/home/yanbc/workspace/codes/scenario_runner \
DISPLAY=:1.0 \
SCENARIO_RUNNER_ROOT=/data/workspace/home/yanbc/workspace/codes/scenario_runner \
python carla_run_vehicle.py scenario_configs/NoSignalJunctionCrossing.json --carla 172.17.0.3 --apollo 172.17.0.2 --show
```


## TO-DO
1. include scenario_runner as a git submodule, and start scenerio_manager as a child process
2. turn all `print`s into `log`s
3. project docker image (Dockerfile)
4. apollo opendrive map converter (*.xodr -> apollo_map)


## About Apollo Map
- Apollo Autopilot maps use a modified version of opendrive, so directly export carla maps to opendrive format won't be compatible with Apollo.
- AuroAI has converted all carla maps to Apollo maps format, [see here](https://auro.ai/blog/2020/03/using-open-source-frameworks-in-autonomous-vehicle-development-part-2/)
