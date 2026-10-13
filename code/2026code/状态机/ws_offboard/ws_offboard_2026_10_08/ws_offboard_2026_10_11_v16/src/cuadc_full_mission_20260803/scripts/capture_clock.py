"""Convert original RealSense frame metadata to the host/ROS clock.

Never invent an offset from USB arrival time. global_time must be available;
sensor_timestamp - frame_timestamp moves the SDK frame timestamp to exposure
midpoint. Both metadata counters are microseconds and wrap at 2**32.
"""
import math


def exposure_midpoint_ns(global_ms, sensor_us, frame_us):
    if not math.isfinite(global_ms) or global_ms <= 0:
        raise ValueError("invalid RealSense global timestamp")
    delta_us = (int(sensor_us) - int(frame_us) + 2**31) % 2**32 - 2**31
    if abs(delta_us) > 100_000:
        raise ValueError("invalid exposure/frame timestamp difference")
    return round(global_ms * 1_000_000) + delta_us * 1000


class CaptureClock:
    def __init__(self):
        self.last_ns = None
        self.last_host_ns = None
        self.last_mono_ns = None

    def read(self, frames, rs, host_ns, ros_ns, mono_ns):
        if abs(ros_ns - host_ns) > 50_000_000:
            raise ValueError("ROS and host clocks differ; simulated time is unsupported")
        if self.last_host_ns is not None:
            jump = (host_ns - self.last_host_ns) - (mono_ns - self.last_mono_ns)
            if abs(jump) > 20_000_000:
                raise ValueError("host clock stepped during capture")
        values = {}
        stamps = []
        for name, frame in (("color", frames.get_color_frame()),
                            ("depth", frames.get_depth_frame())):
            if not frame or frame.get_frame_timestamp_domain() != rs.timestamp_domain.global_time:
                raise ValueError(name + " global_time not ready")
            md = {}
            for key in ("sensor_timestamp", "frame_timestamp", "actual_exposure"):
                kind = getattr(rs.frame_metadata_value, key)
                md[key] = frame.get_frame_metadata(kind) if frame.supports_frame_metadata(kind) else None
            if md["sensor_timestamp"] is None or md["frame_timestamp"] is None:
                raise ValueError(name + " exposure timestamp metadata unavailable")
            stamp = exposure_midpoint_ns(frame.get_timestamp(), md["sensor_timestamp"], md["frame_timestamp"])
            age = (host_ns - stamp) / 1e6
            if not -5 <= age <= 250:
                raise ValueError(f"{name} acquisition age out of bounds: {age:.2f}ms")
            stamps.append(stamp)
            values.update({name + "_" + k: v for k, v in md.items()})
            values[name + "_acquisition_age_ms"] = round(age, 3)
        skew_ms = (stamps[0] - stamps[1]) / 1e6
        if abs(skew_ms) > 25:
            raise ValueError(f"RGB/depth exposure skew too large: {skew_ms:.2f}ms")
        if self.last_ns is not None and stamps[0] <= self.last_ns:
            raise ValueError("duplicate/backwards color exposure timestamp")
        self.last_ns = stamps[0]
        self.last_host_ns, self.last_mono_ns = host_ns, mono_ns
        values["rgb_depth_skew_ms"] = round(skew_ms, 3)
        values["capture_time_source"] = "global_time+sensor_midpoint"
        return stamps[0], values
