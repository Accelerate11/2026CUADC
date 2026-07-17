# Third-Party Notices

The project license does not override third-party terms.

## ArduPilot and MAVROS

ArduPilot SITL and MAVROS are runtime dependencies and are installed separately.

## ardupilot_gazebo Iris assets

`models/iris_d435i/` and `models/iris_d435i_airframe/` include modified model descriptions and Iris meshes derived from `ardupilot_gazebo`. The local source package declares GNU LGPL v3.0. Preserve upstream notices and verify the upstream repository when redistributing modified assets.

## Hazard marker textures

PNG files under `models/hazard_marker/materials/textures/` were prepared from CUADC competition attachment 11 material supplied for this project. They are included for competition simulation, research and training reproducibility.

The project does not claim these files as original artwork, and they are not automatically relicensed under LGPL-3.0-only. Before mirroring or commercial redistribution, verify the competition material's publication terms. If necessary, replace them with independently licensed textures while preserving filenames and class mapping.

## ONNX model

The trained `dangerous_target.onnx` is intentionally excluded. Users must provide a model for which they hold appropriate rights and set `CUADC_HAZARD_MODEL`.

ROS 2, Gazebo Sim, OpenCV, ONNX Runtime, yaml-cpp and NumPy retain their own licenses.
