## Compile protos
```bash
protoc -I . --python_out . modules/perception/proto/traffic_light_detection.proto
```

## Run control cmd demo
```bash
python control_cmd_demo.py --carla 172.17.0.2 --apollo 172.17.0.3 --sync --loop
```
