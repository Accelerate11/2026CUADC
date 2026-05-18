import cv2
import numpy as np
import argparse
import csv
import time
import os
from datetime import datetime
import onnxruntime as ort

# ===================== 全局配置 =====================
# 中文拼音标签（严格对应原10类YOLO标签，无中文乱码风险）
CLASS_NAMES = [
    "BaoZhaPin", "ShengWuWeiHai", "YiRan", "FangSheXingWuPin", "BuRanQiTi",
    "FuShiPin", "YouDuPin", "YuShiYiRanWuPin", "ZiRanWuPin", "CiJiXing"
]
MIN_CONSECUTIVE_CONFIRM = 5
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
TARGET_FPS = 15
# ONNX模型路径
DEFAULT_ONNX_MODEL = "dangerous_target.onnx"
DEFAULT_SOURCE = "0"  # Ubuntu摄像头默认索引0
DEFAULT_CONF = 0.50
DEFAULT_IOU = 0.45
# =====================================================

# ===================== YOLO ONNX推理类 =====================
class YOLOONNX:
    def __init__(self, model_path, conf_thres=0.5, iou_thres=0.45):
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        
        # 初始化ONNX Runtime（优先CPU，适配边缘设备）
        providers = ['CPUExecutionProvider']
        self.session = ort.InferenceSession(model_path, providers=providers)
        
        # 获取输入输出信息
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        self.input_shape = self.session.get_inputs()[0].shape
        self.input_height = self.input_shape[2]
        self.input_width = self.input_shape[3]

    def _preprocess(self, img):
        """预处理：尺寸调整+归一化+格式转换"""
        img_resized = cv2.resize(img, (self.input_width, self.input_height))
        img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)
        img_normalized = img_rgb / 255.0
        img_transposed = np.transpose(img_normalized, (2, 0, 1))
        return np.expand_dims(img_transposed, axis=0).astype(np.float32)

    def _postprocess(self, outputs, img_shape):
        """后处理：NMS+坐标还原"""
        predictions = np.squeeze(outputs[0]).T
        
        boxes = []
        scores = []
        class_ids = []
        
        img_h, img_w = img_shape
        x_scale = img_w / self.input_width
        y_scale = img_h / self.input_height
        
        for pred in predictions:
            class_scores = pred[4:]
            class_id = np.argmax(class_scores)
            score = class_scores[class_id]
            
            if score > self.conf_thres:
                x, y, w, h = pred[:4]
                x1 = int((x - w/2) * x_scale)
                y1 = int((y - h/2) * y_scale)
                x2 = int((x + w/2) * x_scale)
                y2 = int((y + h/2) * y_scale)
                
                boxes.append([x1, y1, x2, y2])
                scores.append(float(score))
                class_ids.append(int(class_id))
        
        # NMS非极大值抑制
        if boxes:
            indices = cv2.dnn.NMSBoxes(boxes, scores, self.conf_thres, self.iou_thres)
            if len(indices) > 0:
                indices = indices.flatten()
                return [boxes[i] for i in indices], [scores[i] for i in indices], [class_ids[i] for i in indices]
        
        return [], [], []

    def predict(self, img):
        """完整推理流程"""
        img_shape = img.shape[:2]
        input_tensor = self._preprocess(img)
        outputs = self.session.run([self.output_name], {self.input_name: input_tensor})
        return self._postprocess(outputs, img_shape)

# ===================== 检测管理器 =====================
class DetectionManager:
    def __init__(self):
        self.consecutive_counts = [0] * len(CLASS_NAMES)
        self.confirmed_classes = set()

    def update(self, detected_classes):
        # 连续帧计数逻辑
        for i in range(len(self.consecutive_counts)):
            if i in detected_classes:
                self.consecutive_counts[i] += 1
            else:
                self.consecutive_counts[i] = 0

        # 连续帧达标则确认目标
        for i in range(len(self.consecutive_counts)):
            if self.consecutive_counts[i] >= MIN_CONSECUTIVE_CONFIRM and i not in self.confirmed_classes:
                self.confirmed_classes.add(i)
                print(f"\n[DETECTED] Confirmed Target: {CLASS_NAMES[i]}")

# ===================== 主程序 =====================
def main():
    parser = argparse.ArgumentParser(description="Ubuntu Edge Device - Hazard Detection (Pinyin Version)")
    parser.add_argument("--source", type=str, default=DEFAULT_SOURCE)
    parser.add_argument("--model", type=str, default=DEFAULT_ONNX_MODEL)
    parser.add_argument("--conf", type=float, default=DEFAULT_CONF)
    parser.add_argument("--iou", type=float, default=DEFAULT_IOU)
    args = parser.parse_args()

    # 加载ONNX模型
    try:
        model = YOLOONNX(args.model, conf_thres=args.conf, iou_thres=args.iou)
        print(f"[OK] Loaded ONNX model: {args.model}")
        print(f"[OK] Model Input Size: {model.input_width}x{model.input_height}")
    except Exception as e:
        print(f"[ERROR] Failed to load ONNX model: {e}")
        return

    detector = DetectionManager()
    print(f"[OK] Detection Manager Initialized (Require {MIN_CONSECUTIVE_CONFIRM} Consecutive Frames)")

    # 打开摄像头/视频源
    try:
        cap = cv2.VideoCapture(int(args.source) if args.source.isdigit() else args.source)
        # 固定720P分辨率
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)
    except Exception as e:
        print(f"[ERROR] Failed to open video source: {e}")
        return

    # 检查摄像头是否正常
    ret, frame = cap.read()
    if not ret:
        print("[ERROR] Cannot read camera frame, please check device")
        cap.release()
        return

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"[OK] Camera Resolution: {w}x{h}")
    print("="*60)
    print("Press 'q' to quit the program")
    print("="*60)

    frame_interval = 1.0 / TARGET_FPS
    prev_time = 0

    # 主循环
    while True:
        # 强制锁定15FPS
        now = time.time()
        if now - prev_time < frame_interval:
            time.sleep(frame_interval - (now - prev_time))
        prev_time = time.time()

        ret, frame = cap.read()
        if not ret:
            print("[WARN] Cannot read frame, retrying...")
            time.sleep(0.1)
            continue

        # ONNX推理检测
        boxes, scores, class_ids = model.predict(frame)

        current_classes = []
        # 绘制检测框和拼音标签
        for box, score, cls in zip(boxes, scores, class_ids):
            if 0 <= cls < len(CLASS_NAMES):
                current_classes.append(cls)
                x1, y1, x2, y2 = box
                # 黄色检测框
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)
                # 拼音标签+置信度
                label = f"{CLASS_NAMES[cls]} {score:.2f}"
                cv2.putText(frame, label, (x1, y1-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)

        # 更新检测状态
        detector.update(current_classes)

        # 左侧：已确认目标
        cv2.putText(frame, "Confirmed Targets:", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
        y_off = 60
        for cls in sorted(detector.confirmed_classes):
            cv2.putText(frame, f"[OK] {CLASS_NAMES[cls]}", (10, y_off), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            y_off += 35

        # 右侧：实时检测进度
        cv2.putText(frame, "Detection Progress:", (700, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,0,0), 2)
        y_off_debug = 60
        for i in range(len(CLASS_NAMES)):
            cnt = detector.consecutive_counts[i]
            if cnt > 0 or i in detector.confirmed_classes:
                color = (0,255,0) if i in detector.confirmed_classes else (255,0,0)
                status = "CONFIRMED" if i in detector.confirmed_classes else f"{cnt}/{MIN_CONSECUTIVE_CONFIRM}"
                cv2.putText(frame, f"{CLASS_NAMES[i]}: {status}", (700, y_off_debug), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                y_off_debug += 25

        # 显示画面
        cv2.imshow("Hazard Detection (Ubuntu Edge)", frame)
        # 按q键退出（Ubuntu通用，避免ESC键兼容性问题）
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("\n[OK] User pressed 'q', exiting program...")
            break

    # 释放资源
    cap.release()
    cv2.destroyAllWindows()

    # ===================== 保存CSV结果 =====================
    print("\n[SAVING] Saving detection results...")
    table_data = [["Hazard Type (Pinyin)", "Status"]]
    for cls in sorted(detector.confirmed_classes):
        table_data.append([CLASS_NAMES[cls], "Confirmed"])
    table_data.append(["Total", len(detector.confirmed_classes)])

    csv_saved = False
    try:
        # 优先保存到当前目录（带时间戳）
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_filename = f"hazard_detection_results_{timestamp}.csv"
        with open(csv_filename, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerows(table_data)
        print(f"[OK] Results saved to: {csv_filename}")
        csv_saved = True
    except Exception as e:
        print(f"[WARN] Save to current directory failed: {e}")
        try:
            # 备用：保存到Ubuntu桌面
            desktop_path = os.path.join(os.path.expanduser("~"), "Desktop")
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            csv_filename = os.path.join(desktop_path, f"hazard_detection_results_{timestamp}.csv")
            with open(csv_filename, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerows(table_data)
            print(f"[OK] Results saved to Desktop: {csv_filename}")
            csv_saved = True
        except Exception as e2:
            print(f"[ERROR] Save to Desktop also failed: {e2}")

    # ===================== 控制台打印最终结果 =====================
    print("\n" + "="*60)
    print("FINAL DETECTION RESULTS")
    print("="*60)
    if detector.confirmed_classes:
        for cls in sorted(detector.confirmed_classes):
            print(f"[OK] {CLASS_NAMES[cls]}")
    else:
        print("[WARN] No hazard targets detected")
    print(f"Total Confirmed Targets: {len(detector.confirmed_classes)}")
    print("="*60)

if __name__ == "__main__":
    main()