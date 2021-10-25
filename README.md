## Compile protos
```bash
bash compile_proto.sh
```

## TO-DO
1. include scenario_runner as a git submodule, and start scenerio_manager as a child process
2. turn all `print`s into `log`s
3. project docker image (Dockerfile)
4. apollo opendrive map converter (*.xodr -> apollo_map)


## About Apollo Map
- Apollo Autopilot maps use a modified version of opendrive, so directly export carla maps to opendrive format won't be compatible with Apollo.
- AuroAI has converted all carla maps to Apollo maps format, [see here](https://auro.ai/blog/2020/03/using-open-source-frameworks-in-autonomous-vehicle-development-part-2/)
