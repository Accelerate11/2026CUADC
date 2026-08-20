# Perception licensing and model provenance

- `models/basket_v3.pt` is the user-supplied Ultralytics segmentation checkpoint.
- Trusted SHA-256: `061b67ace71d5f036f7003a3699640dc0a8522e3c3162116884b67b552cd87bf`.
- The checkpoint metadata reports Ultralytics `8.4.104` and license field
  `AGPL-3.0`; deployment must comply with the applicable Ultralytics terms.
- `basket_detect_seg_analysis.py` is the supplied analysis implementation and
  remains beside the ROS adapter for traceability. The flight entry point uses
  the transport-independent subset in `bucket_algorithms.py`; it does not import
  the analysis script or open a camera through `pyrealsense2`.

The hash and metadata are provenance checks, not an accuracy or airworthiness
claim. Only the exact trusted checkpoint is accepted by preflight and runtime.
