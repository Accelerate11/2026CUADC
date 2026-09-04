# Notice

This package contains a derived Gazebo Iris model.

Derived model files:

```text
models/iris_d435i
models/iris_d435i_airframe
```

Source:

```text
ArduPilot/ardupilot_gazebo
local path used during development: ~/ardupilot_gazebo
upstream license file: ardupilot_gazebo/LICENSE.md
license observed locally: GNU Lesser General Public License v3
```

Local modifications:

- renamed derived models to `iris_d435i` and `iris_d435i_airframe`
- added a downward RGB-D camera sensor for D435i-like simulation
- preserved the ArduPilot plugin control logic, rotor layout, mass, inertia, and motor parameters

The hazard marker textures under:

```text
models/hazard_marker/materials/textures
```

were generated from the CUADC competition attachment 11 hazard chemical marker PDF supplied for this project. They are intended for competition simulation and training use.
