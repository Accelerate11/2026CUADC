#!/usr/bin/env python3
"""仅用于地面的 D435i 相机检查，提供及时响应的实时窗口。"""

import argparse
import importlib
import os
import sys
import time


def main() -> int:
    parser = argparse.ArgumentParser(description="Show the D435i camera for ground preflight")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--exposure", type=float, default=0.0,
                        help="manual exposure; 0 selects camera auto exposure")
    parser.add_argument("exposure_positional", nargs="?", type=float,
                        help="same as --exposure, for .sh NUMBER convention")
    args = parser.parse_args()
    if args.exposure_positional is not None:
        args.exposure = args.exposure_positional
    if args.exposure < 0.0:
        parser.error("exposure must be zero (auto) or positive")
    if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        print("FATAL: no graphical display; run this on the ground station desktop", file=sys.stderr)
        return 2
    helper = importlib.import_module("cuadc_perception.basket_detect_seg_analysis")
    config = argparse.Namespace(
        width=args.width, height=args.height, fps=args.fps,
        depth_width=848, depth_height=480,
    )
    pipeline = profile = None
    try:
        devices = helper.rs.context().query_devices()
        if len(devices) == 0:
            raise RuntimeError("no RealSense device detected")
        device = devices[0]
        helper.require_usb3(device)
        helper.configure_camera(device, args.exposure)
        pipeline, profile, color_format, format_name = helper.start_rgbd_stream(config)
        cv2 = helper.cv2
        cv2.namedWindow("CUADC camera preflight", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("CUADC camera preflight", 960, 540)
        print("[OK] camera streaming; press q or Esc to close")
        frames = 0
        started = time.monotonic()
        while True:
            frameset = pipeline.wait_for_frames(1500)
            color = frameset.get_color_frame()
            if not color:
                continue
            image = helper.frame_to_bgr(color, color_format)
            frames += 1
            elapsed = max(0.001, time.monotonic() - started)
            cv2.putText(image, "D435i {}  {:.1f} FPS  q/Esc: close".format(
                format_name, frames / elapsed), (18, 32),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.imshow("CUADC camera preflight", image)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            if cv2.getWindowProperty("CUADC camera preflight", cv2.WND_PROP_VISIBLE) < 1:
                break
        return 0
    except Exception as error:
        print("FATAL: camera preflight failed: {}".format(error), file=sys.stderr)
        return 1
    finally:
        if pipeline is not None:
            try:
                pipeline.stop()
            except Exception:
                pass
        try:
            helper.cv2.destroyAllWindows()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
