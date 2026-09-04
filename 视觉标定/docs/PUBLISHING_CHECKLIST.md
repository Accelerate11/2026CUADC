# GitHub Publishing Checklist

- [ ] Choose and add the final open-source license; update `package.xml` and `setup.py`.
- [ ] Copy `config/aircraft.example.yaml` only; never commit `config/aircraft.yaml`.
- [ ] Confirm `git status` contains no `logs/`, `build/`, `install/`, serial numbers or FCU device paths.
- [ ] Confirm no model/checkpoint files (`*.pt`, `*.onnx`, `*.engine`) were added accidentally.
- [ ] Run `python3 -m py_compile` / unit tests.
- [ ] On a ROS 2 Humble machine, run `./bin/build.sh` and `./bin/check_vision_contract.sh` with a real provider.
