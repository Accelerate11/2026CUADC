# License and provenance notes

- `basket_v3.pt` and `basket_detect_seg_analysis.py` were supplied by the user on 2026-08-03. Their original SHA-256 values are recorded in the delivery manifest.
- Static checkpoint metadata reports Ultralytics 8.4.104 and `AGPL-3.0`. Deploying the Ultralytics runtime or distributing a derivative system may create AGPL obligations; obtain project-specific legal review if the surrounding flight stack is not intended for AGPL distribution.
- A PyTorch `.pt` checkpoint is a pickle-based container and can execute code during deserialization. The runtime must load only the checksum-verified model delivered with this package.
- ArduPilot, MAVROS, ROS 2, Gazebo, RealSense, PyTorch and other runtime components retain their own licenses. They are dependencies, not relicensed by this package.
- Newly written mission glue, launch files, deployment scripts and documentation are provided for this user's project. No warranty of airworthiness or fitness for flight is made.
