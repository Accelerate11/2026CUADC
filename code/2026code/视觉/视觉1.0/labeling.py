# 图像标注工具 - 完整版
# 功能：左侧缩略图 + 右侧拉框标注 + 精修/移动/删除框 + 选择.pt权重 + 保存标注
# 安装依赖：pip install pyqt5 pillow torch
import sys
import os
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
                             QPushButton, QFileDialog, QListWidget, QListWidgetItem,
                             QGraphicsView, QGraphicsScene, QGraphicsRectItem, QLabel,
                             QMessageBox, QSplitter, QFrame)
from PyQt5.QtGui import QPixmap, QPen, QBrush, QColor
from PyQt5.QtCore import Qt, QRectF, QPointF
from PIL import Image
import torch

class AnnotationCanvas(QGraphicsView):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.scene = QGraphicsScene(self)
        self.setScene(self.scene)
        self.setRenderHint(self.renderHints().Antialiasing)
        self.setMouseTracking(True)
        self.setDragMode(QGraphicsView.NoDrag)

        self.current_pixmap = None
        self.original_size = None
        self.annotations = []
        self.selected_box = None
        self.is_drawing = False
        self.is_moving = False
        self.is_resizing = False
        self.start_point = QPointF()
        self.current_rect = None
        self.resize_handle = None

        self.box_pen = QPen(QColor(255, 0, 0), 2)
        self.selected_pen = QPen(QColor(0, 255, 0), 3)

    def load_image(self, image_path):
        self.clear_all()
        self.current_pixmap = QPixmap(image_path)
        self.original_size = (self.current_pixmap.width(), self.current_pixmap.height())
        scaled_pixmap = self.current_pixmap.scaled(self.viewport().size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.scene.addPixmap(scaled_pixmap)
        self.setSceneRect(QRectF(scaled_pixmap.rect()))
        self.load_annotations(image_path)

    def load_annotations(self, image_path):
        txt_path = os.path.splitext(image_path)[0] + '.txt'
        if os.path.exists(txt_path):
            with open(txt_path, 'r') as f:
                for line in f.readlines():
                    parts = line.strip().split()
                    if len(parts) >= 4:
                        x1, y1, x2, y2 = map(float, parts[:4])
                        sx = self.scene.width() / self.original_size[0]
                        sy = self.scene.height() / self.original_size[1]
                        self.add_box(x1*sx, y1*sy, x2*sx, y2*sy)

    def save_annotations(self, image_path):
        if not self.annotations:
            QMessageBox.information(self, "提示", "无标注可保存")
            return
        txt_path = os.path.splitext(image_path)[0] + '.txt'
        with open(txt_path, 'w') as f:
            sx = self.original_size[0] / self.scene.width()
            sy = self.original_size[1] / self.scene.height()
            for box in self.annotations:
                f.write(f"{box[0]*sx:.2f} {box[1]*sy:.2f} {box[2]*sx:.2f} {box[3]*sy:.2f}\n")
        QMessageBox.information(self, "成功", f"标注已保存：{txt_path}")

    def add_box(self, x1, y1, x2, y2):
        rect = QGraphicsRectItem(QRectF(x1, y1, x2-x1, y2-y1))
        rect.setPen(self.box_pen)
        self.scene.addItem(rect)
        self.annotations.append([x1, y1, x2, y2])
        return rect

    def delete_selected_box(self):
        if self.selected_box:
            idx = self.scene.items().index(self.selected_box) - 1
            if 0 <= idx < len(self.annotations):
                del self.annotations[idx]
            self.scene.removeItem(self.selected_box)
            self.selected_box = None

    def clear_all(self):
        self.scene.clear()
        self.annotations = []
        self.selected_box = None
        self.current_pixmap = None
        self.original_size = None

    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton and self.current_pixmap:
            p = self.mapToScene(e.pos())
            if self.selected_box:
                self.is_resizing = True
                self.start_point = p
                return
            for item in self.scene.items(p):
                if isinstance(item, QGraphicsRectItem):
                    if self.selected_box:
                        self.selected_box.setPen(self.box_pen)
                    self.selected_box = item
                    self.selected_box.setPen(self.selected_pen)
                    self.is_moving = True
                    self.start_point = p
                    return
            self.is_drawing = True
            self.start_point = p
            self.current_rect = QGraphicsRectItem()
            self.current_rect.setPen(self.box_pen)
            self.scene.addItem(self.current_rect)
            if self.selected_box:
                self.selected_box.setPen(self.box_pen)
                self.selected_box = None

    def mouseMoveEvent(self, e):
        if not self.current_pixmap:
            return
        p = self.mapToScene(e.pos())
        if self.is_drawing and self.current_rect:
            self.current_rect.setRect(QRectF(self.start_point, p).normalized())
        elif self.is_moving and self.selected_box:
            dx = p.x() - self.start_point.x()
            dy = p.y() - self.start_point.y()
            r = self.selected_box.rect()
            r.translate(dx, dy)
            self.selected_box.setRect(r)
            self.start_point = p
            idx = self.scene.items().index(self.selected_box) - 1
            if 0 <= idx < len(self.annotations):
                self.annotations[idx] = [r.left(), r.top(), r.right(), r.bottom()]

    def mouseReleaseEvent(self, e):
        if e.button() == Qt.LeftButton:
            if self.is_drawing and self.current_rect:
                r = self.current_rect.rect()
                if r.width() > 5 and r.height() > 5:
                    self.annotations.append([r.left(), r.top(), r.right(), r.bottom()])
                else:
                    self.scene.removeItem(self.current_rect)
                self.current_rect = None
            self.is_drawing = False
            self.is_moving = False
            self.is_resizing = False

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("图像标注工具 - 支持.pt权重")
        self.setGeometry(100, 100, 1300, 800)
        self.image_paths = []
        self.current_idx = -1
        self.model = None
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.init_ui()

    def init_ui(self):
        cw = QWidget()
        self.setCentralWidget(cw)
        main_layout = QHBoxLayout(cw)
        split = QSplitter(Qt.Horizontal)

        # 左侧
        left = QWidget()
        left_layout = QVBoxLayout(left)
        btn1 = QPushButton("导入图片")
        btn1.clicked.connect(self.import_imgs)
        btn2 = QPushButton("删除当前图")
        btn2.clicked.connect(self.del_img)
        left_layout.addWidget(btn1)
        left_layout.addWidget(btn2)
        self.list = QListWidget()
        self.list.setIconSize(Qt.QSize(130, 100))
        self.list.setViewMode(QListWidget.IconMode)
        self.list.itemClicked.connect(self.sel_img)
        left_layout.addWidget(self.list)
        split.addWidget(left)
        split.setStretchFactor(0, 1)

        # 右侧
        right = QWidget()
        right_layout = QVBoxLayout(right)
        model_frame = QFrame()
        model_layout = QHBoxLayout(model_frame)
        self.model_lab = QLabel("模型：未加载")
        model_layout.addWidget(self.model_lab)
        load_pt = QPushButton("选择 .pt 权重")
        load_pt.clicked.connect(self.load_pt)
        model_layout.addWidget(load_pt)
        right_layout.addWidget(model_frame)
        self.canvas = AnnotationCanvas()
        right_layout.addWidget(self.canvas)
        # 底部按钮
        btl = QHBoxLayout()
        btl.addWidget(QPushButton("删除选中框", clicked=self.canvas.delete_selected_box))
        btl.addWidget(QPushButton("清空所有框", clicked=self.canvas.clear_all))
        btl.addWidget(QPushButton("保存标注", clicked=self.save_ann))
        right_layout.addLayout(btl)
        split.addWidget(right)
        split.setStretchFactor(1, 4)
        main_layout.addWidget(split)

    def import_imgs(self):
        fs, _ = QFileDialog.getOpenFileNames(self, "选图", "", "Images (*.png *.jpg *.jpeg *.bmp)")
        if fs:
            self.image_paths.extend(fs)
            self.update_list()

    def update_list(self):
        self.list.clear()
        for p in self.image_paths:
            item = QListWidgetItem(os.path.basename(p))
            item.setIcon(Qt.QIcon(QPixmap(p).scaled(130, 100, Qt.KeepAspectRatio)))
            self.list.addItem(item)

    def sel_img(self, item):
        self.current_idx = self.list.row(item)
        if 0 <= self.current_idx < len(self.image_paths):
            self.canvas.load_image(self.image_paths[self.current_idx])

    def del_img(self):
        if self.current_idx >= 0:
            del self.image_paths[self.current_idx]
            self.update_list()
            self.canvas.clear_all()
            self.current_idx = -1

    def save_ann(self):
        if self.current_idx >= 0:
            self.canvas.save_annotations(self.image_paths[self.current_idx])

    def load_pt(self):
        path, _ = QFileDialog.getOpenFileName(self, "选择.pt文件", "", "Model (*.pt *.pth)")
        if path:
            try:
                self.model = torch.load(path, map_location=self.device)
                self.model.eval()
                self.model_lab.setText(f"模型：{os.path.basename(path)}")
                QMessageBox.information(self, "成功", "权重加载完成")
            except Exception as e:
                QMessageBox.critical(self, "失败", f"加载错误：{str(e)}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())