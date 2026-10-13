import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from basket_vision_ros_node import BasketVisionRosNode, openvino_model_for_onnx


class FakeLogger:
    def __init__(self):
        self.errors = []

    def info(self, message):
        pass

    def error(self, message):
        self.errors.append(message)


class FakeModel:
    def __init__(self, path):
        self.path = path
        self.fail_next = False
        self.calls = []

    def __call__(self, image, **kwargs):
        if self.fail_next:
            self.fail_next = False
            raise RuntimeError("GPU unavailable")
        self.calls.append(kwargs)
        return [self.path]


class TestVisionBackend(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.source = Path(self.temp.name) / "basket_detect.onnx"
        self.source.touch()
        self.models = {}
        self.logger = FakeLogger()

    def node(self, device):
        node = BasketVisionRosNode.__new__(BasketVisionRosNode)
        node.model_path = str(self.source)
        node.cpu_model_path = str(self.source)
        node.device = device
        node.requested_device = device
        node.target_class = 0
        node.confidence_threshold = 0.08
        node.nms_iou_threshold = 0.01
        node.inference_image_size = 640
        node.half = False
        node.using_tensorrt = False
        node.node = SimpleNamespace(get_logger=lambda: self.logger)

        def create_model(path, task):
            self.assertEqual(task, "detect")
            model = FakeModel(path)
            self.models[path] = model
            return model

        node.vision = SimpleNamespace(YOLO=create_model)
        return node

    def create_gpu_files(self):
        path = openvino_model_for_onnx(self.source)
        path.parent.mkdir()
        path.touch()
        path.with_suffix(".bin").touch()
        (path.parent / "metadata.yaml").touch()
        return path

    def test_cpu_uses_original_model(self):
        node = self.node("cpu")
        node._load_model()
        self.assertEqual(node.model_path, str(self.source))
        self.assertEqual(node._run_inference(None), str(self.source))
        self.assertEqual(self.models[str(self.source)].calls[0]["device"], "cpu")

    def test_gpu_uses_ir_and_warms_up(self):
        gpu_path = self.create_gpu_files()
        node = self.node("intel:gpu")
        node._load_model()
        self.assertEqual(node.model_path, str(gpu_path))
        self.assertEqual(node.device, "intel:gpu")
        self.assertEqual(len(self.models[str(gpu_path)].calls), 1)
        self.assertEqual(self.models[str(gpu_path)].calls[0]["device"], "intel:gpu")

    def test_missing_ir_falls_back_to_cpu(self):
        node = self.node("intel:gpu")
        node._load_model()
        self.assertEqual(node.device, "cpu")
        self.assertEqual(node.model_path, str(self.source))
        self.assertIn("initialization failed", self.logger.errors[0])

    def test_runtime_gpu_error_falls_back_once(self):
        gpu_path = self.create_gpu_files()
        node = self.node("intel:gpu")
        node._load_model()
        self.models[str(gpu_path)].fail_next = True
        self.assertEqual(node._run_inference(None), str(self.source))
        self.assertEqual(node.device, "cpu")
        self.assertEqual(node._run_inference(None), str(self.source))
        self.assertEqual(len(self.logger.errors), 1)


if __name__ == "__main__":
    unittest.main()
