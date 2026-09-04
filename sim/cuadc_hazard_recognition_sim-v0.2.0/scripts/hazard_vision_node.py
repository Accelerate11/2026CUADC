#!/usr/bin/env python3
"""ROS 2 adapter and live GUI for the user-provided CUADC hazard ONNX detector."""

from __future__ import annotations

import os
import time
from pathlib import Path
from typing import List, Tuple

import cv2
from cv_bridge import CvBridge
import numpy as np
import onnxruntime as ort
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from cuadc_hazard_recognition_sim.msg import HazardDetection


CLASS_NAMES = [
    "BaoZhaPin", "ShengWuWeiHai", "YiRan", "FangSheXingWuPin", "BuRanQiTi",
    "FuShiPin", "YouDuPin", "YuShiYiRanWuPin", "ZiRanWuPin", "CiJiXing",
]


def iou(a: List[float], b: List[float]) -> float:
    left, top = max(a[0], b[0]), max(a[1], b[1])
    right, bottom = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, right - left) * max(0.0, bottom - top)
    area_a = max(0.0, a[2] - a[0]) * max(0.0, a[3] - a[1])
    area_b = max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])
    return inter / max(1.0e-9, area_a + area_b - inter)


def class_aware_nms(
    boxes: List[List[float]], scores: List[float], classes: List[int], threshold: float
) -> List[int]:
    keep: List[int] = []
    for index in sorted(range(len(scores)), key=lambda i: scores[i], reverse=True):
        if any(classes[index] == classes[j] and iou(boxes[index], boxes[j]) > threshold for j in keep):
            continue
        keep.append(index)
    return keep


class HazardVisionNode(Node):
    def __init__(self) -> None:
        super().__init__("hazard_vision_node")
        self.declare_parameter("model_path", os.environ.get("CUADC_HAZARD_MODEL", ""))
        self.declare_parameter("image_topic", "/uav/d435i/color/image")
        self.declare_parameter("output_topic", "/perception/hazard_detection")
        self.declare_parameter("confidence", 0.50)
        self.declare_parameter("iou", 0.45)
        self.declare_parameter("min_consecutive_confirm", 5)
        self.declare_parameter("max_inference_fps", 15.0)
        self.declare_parameter("quarter_turn_tta", True)
        self.declare_parameter("tta_merge_iou", 0.60)
        self.declare_parameter("show_window", True)
        self.declare_parameter("window_name", "CUADC Hazard Recognition - RTK Recon")
        self.declare_parameter("publish_annotated", True)
        self.declare_parameter("annotated_topic", "/perception/hazard_annotated")
        self.declare_parameter("debug_image_dir", "")
        self.declare_parameter("debug_save_every_n", 15)
        self.declare_parameter("debug_save_annotated", True)

        gp = self.get_parameter
        self.confidence = float(gp("confidence").value)
        self.iou_threshold = float(gp("iou").value)
        self.min_consecutive = max(1, int(gp("min_consecutive_confirm").value))
        self.min_period = 1.0 / max(0.5, float(gp("max_inference_fps").value))
        self.quarter_turn_tta = bool(gp("quarter_turn_tta").value)
        self.tta_merge_iou = float(gp("tta_merge_iou").value)
        self.show_window = bool(gp("show_window").value)
        self.window_name = str(gp("window_name").value)
        self.publish_annotated = bool(gp("publish_annotated").value)
        self.annotated_topic = str(gp("annotated_topic").value)
        model_path = str(gp("model_path").value).strip()
        if not model_path:
            raise RuntimeError(
                "No ONNX model configured. Set CUADC_HAZARD_MODEL or pass model_path."
            )
        if not Path(model_path).is_file():
            raise FileNotFoundError(f"ONNX model not found: {model_path}")
        debug_dir = str(gp("debug_image_dir").value).strip()
        self.debug_dir = Path(debug_dir) if debug_dir else None
        self.debug_save_every_n = max(1, int(gp("debug_save_every_n").value))
        self.debug_save_annotated = bool(gp("debug_save_annotated").value)
        if self.debug_dir is not None:
            self.debug_dir.mkdir(parents=True, exist_ok=True)

        self.session = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])
        model_input = self.session.get_inputs()[0]
        self.input_name = model_input.name
        self.output_name = self.session.get_outputs()[0].name
        self.input_h = int(model_input.shape[2])
        self.input_w = int(model_input.shape[3])
        self.bridge = CvBridge()
        self.last_inference = 0.0
        self.last_class = -1
        self.consecutive = 0
        self.missed_frames = 0
        self.frame_count = 0
        self.confirmed_class = -1
        self.confirmed_confidence = 0.0
        self.window_initialized = False
        self.last_frame_wall_time = time.monotonic()
        self.display_fps = 0.0
        self.last_class_vectors: List[List[float]] = []

        if self.show_window and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
            self.get_logger().warning(
                "show_window=true but no DISPLAY/WAYLAND_DISPLAY; GUI disabled, inference continues"
            )
            self.show_window = False

        qos = rclpy.qos.QoSProfile(
            depth=2,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
        )
        image_topic = str(gp("image_topic").value)
        output_topic = str(gp("output_topic").value)
        self.publisher = self.create_publisher(HazardDetection, output_topic, 10)
        self.annotated_publisher = (
            self.create_publisher(Image, self.annotated_topic, 5)
            if self.publish_annotated else None
        )
        self.subscription = self.create_subscription(Image, image_topic, self.image_cb, qos)
        self.get_logger().info(
            f"Loaded {model_path}; image={image_topic}, output={output_topic}, "
            f"input={self.input_w}x{self.input_h}, quarter_turn_tta={self.quarter_turn_tta}, "
            f"show_window={self.show_window}, annotated={self.annotated_topic}"
        )

    def infer_single(
        self, image: np.ndarray
    ) -> Tuple[List[List[float]], List[float], List[int], List[List[float]]]:
        image_h, image_w = image.shape[:2]
        resized = cv2.resize(image, (self.input_w, self.input_h), interpolation=cv2.INTER_LINEAR)
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
        tensor = np.transpose(rgb.astype(np.float32) / 255.0, (2, 0, 1))[None, ...]
        raw = self.session.run([self.output_name], {self.input_name: tensor})[0]
        predictions = raw[0].T if raw.shape[1] < raw.shape[2] else raw[0]

        boxes: List[List[float]] = []
        scores: List[float] = []
        classes: List[int] = []
        class_vectors: List[List[float]] = []
        sx, sy = image_w / self.input_w, image_h / self.input_h
        for prediction in predictions:
            class_scores = np.asarray(
                prediction[4:4 + len(CLASS_NAMES)], dtype=np.float32
            )
            class_id = int(np.argmax(class_scores))
            score = float(class_scores[class_id])
            if score < self.confidence:
                continue
            cx, cy, width, height = map(float, prediction[:4])
            boxes.append([
                (cx - width * 0.5) * sx,
                (cy - height * 0.5) * sy,
                (cx + width * 0.5) * sx,
                (cy + height * 0.5) * sy,
            ])
            scores.append(score)
            classes.append(class_id)
            class_vectors.append(class_scores.tolist())

        keep = class_aware_nms(boxes, scores, classes, self.iou_threshold)
        return (
            [boxes[i] for i in keep],
            [scores[i] for i in keep],
            [classes[i] for i in keep],
            [class_vectors[i] for i in keep],
        )

    @staticmethod
    def undo_quarter_turn_box(
        box: List[float], original_width: int, original_height: int, clockwise: bool
    ) -> List[float]:
        """Map a box from a 90-degree rotated image back to source-image pixels."""
        corners = ((box[0], box[1]), (box[2], box[1]), (box[2], box[3]), (box[0], box[3]))
        if clockwise:
            source = [(yr, original_height - 1.0 - xr) for xr, yr in corners]
        else:
            source = [(original_width - 1.0 - yr, xr) for xr, yr in corners]
        xs, ys = zip(*source)
        return [
            max(0.0, min(xs)),
            max(0.0, min(ys)),
            min(original_width - 1.0, max(xs)),
            min(original_height - 1.0, max(ys)),
        ]

    def infer(self, image: np.ndarray) -> Tuple[List[List[float]], List[float], List[int]]:
        if not self.quarter_turn_tta:
            boxes, scores, classes, vectors = self.infer_single(image)
            self.last_class_vectors = vectors
            return boxes, scores, classes

        image_h, image_w = image.shape[:2]
        all_boxes: List[List[float]] = []
        all_class_vectors: List[List[float]] = []
        all_branches: List[int] = []
        for branch, (rotated, clockwise) in enumerate((
            (cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE), True),
            (cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE), False),
        )):
            boxes, _, _, class_vectors = self.infer_single(rotated)
            all_boxes.extend([
                self.undo_quarter_turn_box(box, image_w, image_h, clockwise)
                for box in boxes
            ])
            all_class_vectors.extend(class_vectors)
            all_branches.extend([branch] * len(class_vectors))

        # Merge spatially overlapping predictions from both TTA branches before
        # choosing a class. This preserves the complete class-score vectors
        # instead of discarding every branch's runner-up evidence.
        groups = []
        order = sorted(
            range(len(all_boxes)),
            key=lambda index: max(all_class_vectors[index]),
            reverse=True,
        )
        for index in order:
            box = all_boxes[index]
            vector = all_class_vectors[index]
            match = next(
                (
                    group
                    for group in groups
                    if any(iou(box, member) >= self.tta_merge_iou for member in group["boxes"])
                ),
                None,
            )
            if match is None:
                groups.append({
                    "boxes": [box], "vectors": [vector],
                    "branches": [all_branches[index]],
                })
            else:
                match["boxes"].append(box)
                match["vectors"].append(vector)
                match["branches"].append(all_branches[index])

        fused_boxes: List[List[float]] = []
        fused_scores: List[float] = []
        fused_classes: List[int] = []
        fused_class_vectors: List[List[float]] = []
        for group in groups:
            vectors = np.asarray(group["vectors"], dtype=np.float32)
            branch_vectors = []
            for branch in sorted(set(group["branches"])):
                indices = [
                    index for index, value in enumerate(group["branches"])
                    if value == branch
                ]
                best_index = max(indices, key=lambda index: float(np.max(vectors[index])))
                branch_vectors.append(vectors[best_index])
            aggregate = np.mean(np.asarray(branch_vectors), axis=0)
            # A strong prediction from either rotation triggers and localizes the
            # target. The complete averaged vector is carried separately for
            # the C++ evidence-window classification.
            trigger_index = max(
                range(len(vectors)), key=lambda index: float(np.max(vectors[index]))
            )
            trigger_vector = vectors[trigger_index]
            class_id = int(np.argmax(trigger_vector))
            score = float(trigger_vector[class_id])
            if score < self.confidence:
                continue
            weights = np.maximum(np.max(vectors, axis=1), 1.0e-6)
            box = np.average(np.asarray(group["boxes"], dtype=np.float32), axis=0, weights=weights)
            fused_boxes.append(box.tolist())
            fused_scores.append(score)
            fused_classes.append(class_id)
            fused_class_vectors.append(aggregate.tolist())

        keep = class_aware_nms(
            fused_boxes, fused_scores, fused_classes, self.iou_threshold
        )
        self.last_class_vectors = [fused_class_vectors[i] for i in keep]
        return (
            [fused_boxes[i] for i in keep],
            [fused_scores[i] for i in keep],
            [fused_classes[i] for i in keep],
        )

    def render_annotated(
        self,
        image: np.ndarray,
        boxes: List[List[float]],
        scores: List[float],
        classes: List[int],
        best: int | None,
    ) -> np.ndarray:
        canvas = image.copy()
        for index, (box, score, class_id) in enumerate(zip(boxes, scores, classes)):
            x1, y1, x2, y2 = [int(round(value)) for value in box]
            confirmed = class_id == self.confirmed_class and self.consecutive >= self.min_consecutive
            color = (80, 255, 120) if confirmed else (0, 220, 255)
            thickness = 3 if index == best else 2
            cv2.rectangle(canvas, (x1, y1), (x2, y2), color, thickness)
            label = f"{CLASS_NAMES[class_id]} {score:.2f}"
            cv2.putText(
                canvas, label, (max(0, x1), max(106, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.56, color, 2, cv2.LINE_AA,
            )

        overlay = canvas.copy()
        cv2.rectangle(overlay, (0, 0), (canvas.shape[1], 94), (18, 18, 18), -1)
        cv2.addWeighted(overlay, 0.82, canvas, 0.18, 0.0, canvas)
        if best is None:
            current = "none"
        else:
            current = (
                f"{CLASS_NAMES[classes[best]]} {scores[best]:.2f} "
                f"({self.consecutive}/{self.min_consecutive})"
            )
        confirmed = (
            f"{CLASS_NAMES[self.confirmed_class]} {self.confirmed_confidence:.2f}"
            if self.confirmed_class >= 0 else "none"
        )
        cv2.putText(
            canvas, f"Current: {current}", (12, 29), cv2.FONT_HERSHEY_SIMPLEX,
            0.65, (0, 220, 255), 2, cv2.LINE_AA,
        )
        cv2.putText(
            canvas, f"Confirmed: {confirmed}", (12, 61), cv2.FONT_HERSHEY_SIMPLEX,
            0.65, (80, 255, 120), 2, cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            f"Frame {self.frame_count} | {self.display_fps:.1f} FPS | quarter-turn TTA",
            (12, 86), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (210, 210, 210), 1, cv2.LINE_AA,
        )
        return canvas

    def output_annotated(self, annotated: np.ndarray, source: Image) -> None:
        if self.annotated_publisher is not None:
            message = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            message.header = source.header
            self.annotated_publisher.publish(message)
        if not self.show_window:
            return
        try:
            if not self.window_initialized:
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self.window_name, annotated.shape[1], annotated.shape[0])
                self.window_initialized = True
                self.get_logger().info(f"Recognition window opened: {self.window_name}")
            cv2.imshow(self.window_name, annotated)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                cv2.destroyWindow(self.window_name)
                self.window_initialized = False
                self.show_window = False
                self.get_logger().info("Recognition window closed by user; inference continues")
        except cv2.error as exc:
            self.get_logger().warning(
                f"Cannot display recognition window; GUI disabled, inference continues: {exc}"
            )
            self.show_window = False
            self.window_initialized = False

    def image_cb(self, msg: Image) -> None:
        monotonic = time.monotonic()
        if monotonic - self.last_inference < self.min_period:
            return
        self.last_inference = monotonic
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            boxes, scores, classes = self.infer(image)
            self.frame_count += 1
            frame_wall_time = time.monotonic()
            instantaneous_fps = 1.0 / max(1.0e-6, frame_wall_time - self.last_frame_wall_time)
            self.display_fps = instantaneous_fps if self.display_fps <= 0.0 else (
                0.90 * self.display_fps + 0.10 * instantaneous_fps
            )
            self.last_frame_wall_time = frame_wall_time
        except Exception as exc:
            self.get_logger().error(f"Vision inference failed: {exc}")
            return

        best = None
        if not scores:
            self.last_class = -1
            self.consecutive = 0
            self.missed_frames += 1
            if self.missed_frames >= self.min_consecutive:
                self.confirmed_class = -1
                self.confirmed_confidence = 0.0
            # Publish an explicit empty-frame heartbeat so the C++ temporal
            # filter can reset across missed or intermittent detections.
            detection = HazardDetection()
            detection.header = msg.header
            detection.class_name = ""
            detection.confidence = 0.0
            detection.x1 = detection.y1 = detection.x2 = detection.y2 = 0
            detection.center_u = detection.center_v = 0
            detection.consecutive_count = 0
            detection.confirmed = False
            self.publisher.publish(detection)
        else:
            self.missed_frames = 0
            best = int(np.argmax(scores))
            class_id = classes[best]
            if class_id == self.last_class:
                self.consecutive += 1
            else:
                self.last_class = class_id
                self.consecutive = 1
                self.confirmed_class = -1
                self.confirmed_confidence = 0.0
            if self.consecutive >= self.min_consecutive:
                self.confirmed_class = class_id
                self.confirmed_confidence = max(self.confirmed_confidence, scores[best])

            box = boxes[best]
            detection = HazardDetection()
            detection.header = msg.header
            detection.class_name = CLASS_NAMES[class_id]
            detection.confidence = scores[best]
            detection.x1, detection.y1 = int(box[0]), int(box[1])
            detection.x2, detection.y2 = int(box[2]), int(box[3])
            detection.center_u = (detection.x1 + detection.x2) // 2
            detection.center_v = (detection.y1 + detection.y2) // 2
            detection.consecutive_count = self.consecutive
            detection.confirmed = self.consecutive >= self.min_consecutive
            detection.class_names = list(CLASS_NAMES)
            detection.class_confidences = [
                float(value) for value in self.last_class_vectors[best]
            ]
            self.publisher.publish(detection)

        annotated = self.render_annotated(image, boxes, scores, classes, best)
        if self.debug_dir is not None and self.frame_count % self.debug_save_every_n == 0:
            cv2.imwrite(str(self.debug_dir / f"frame_{self.frame_count:05d}.jpg"), image)
            if self.debug_save_annotated:
                cv2.imwrite(
                    str(self.debug_dir / f"annotated_{self.frame_count:05d}.jpg"), annotated
                )
        self.output_annotated(annotated, msg)
        if self.frame_count % 30 == 0:
            detail = (
                f"{CLASS_NAMES[classes[best]]}:{scores[best]:.3f}"
                if best is not None else "none"
            )
            self.get_logger().info(
                f"Inference frames={self.frame_count}, best={detail}, "
                f"display_fps={self.display_fps:.1f}"
            )

    def destroy_node(self) -> None:
        if self.window_initialized:
            try:
                cv2.destroyWindow(self.window_name)
                cv2.waitKey(1)
            except cv2.error:
                pass
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = HazardVisionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
