# Changelog

## 0.2.0 — 2026-07-17

- Replaced known-bucket point-to-point flight with public-area-only lawnmower coverage.
- Added 7-lane, 14-endpoint coverage of the 5 × 8 m area.
- Added WGS84 → ECEF → RTK ENU → MAVROS local ENU conversion.
- Kept altitude control in MAVROS local Z.
- Added live OpenCV visualization and annotated image publishing.
- Added visual projection, centering and class-evidence accumulation.
- Added complete 10-class vectors, quarter-turn TTA and spatial deduplication.
- Separated Gazebo Server from Gazebo GUI.
- Added post-mission evaluation and explicit ground-truth isolation.
- Verified randomized seeds 4102, 4103 and 4104.
- Removed developer-machine defaults from the GitHub release.
