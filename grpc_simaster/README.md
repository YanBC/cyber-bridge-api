## Compile protos
```bash
# in project root:
python -m grpc_tools.protoc \
    -I . \
    --python_out . \
    --grpc_python_out . \
    grpc_simaster/simaster.proto
```

## Run grpc server
```bash
python start_simaster.py
```

In another terminal, run example client script:
```bash
python grpc_simaster/example_client.py
```
