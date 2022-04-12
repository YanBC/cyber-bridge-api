## Compile protos
`simaster.proto` contains definitions of the simaster grpc service as well as input and output messages.

To compile the corresponding python files:
```bash
# in project root:
python -m grpc_tools.protoc \
    -I . \
    --python_out . \
    --grpc_python_out . \
    grpc_simaster/simaster.proto
```

## Example run
Start simaster server:
```bash
python start_simaster.py
```

In another terminal, run example client script:
```bash
python simaster_client_example.py
```
