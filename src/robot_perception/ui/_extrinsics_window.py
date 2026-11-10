#!/usr/bin/env python3
"""中文固定相机外参工作台。启动不连接 ROS；不控制机器人、不发布 TF。"""
import argparse
import copy
import json
from pathlib import Path
import sys
import threading

import cv2
import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets as W

from ..calibration.common import read_json, require
from ..calibration.extrinsics import defaults, detect, digest, validate_config
from ..calibration.extrinsics_session import Session, load_offline, png_image, validate_packet


class Detector(threading.Thread):
    """One running detection + one replaceable request/output; never touches Qt."""
    def __init__(self):
        super().__init__(name="checkerboard-latest-only", daemon=False)
        self.condition = threading.Condition()
        self.pending = self.output = None
        self.stopping = False
        self.start()

    def submit(self, packet, config, generation):
        with self.condition:
            self.pending = (packet, copy.deepcopy(config), generation)
            self.condition.notify()

    def take(self):
        with self.condition:
            output, self.output = self.output, None
        return output

    def stop(self):
        with self.condition:
            self.stopping = True
            self.pending = None
            self.condition.notify()
        self.join()

    def run(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.stopping or self.pending is not None)
                if self.stopping:
                    return
                packet, config, generation = self.pending
                self.pending = None
            try:
                detection, error = detect(packet["image"], packet["camera_info"], config["board"],
                                          raw_model_confirmed=config["raw_model_confirmed"]), None
            except Exception as exc:
                detection, error = None, str(exc)
            with self.condition:
                self.output = (packet, detection, generation, error)


class Solver(threading.Thread):
    """Solve a private snapshot; main thread accepts only an unchanged input digest."""
    def __init__(self, session, revision):
        super().__init__(name="extrinsics-solver", daemon=False)
        self.session = copy.deepcopy(session)
        self.revision = revision
        self.input_sha256 = digest(session.inputs())
        self.result = self.error = None

    def run(self):
        try:
            self.result = self.session.run_solve()
        except Exception as exc:
            self.error = str(exc)


class Window(W.QMainWindow):
    def __init__(self, session=None):
        super().__init__()
        self.setWindowTitle("固定相机 → robot base 外参 · candidate / 未实装验收")
        self.resize(1220, 900)
        self.session = session or Session()
        self.loading = False
        self.editor_dirty = False
        self.generation = 0
        self.packet = self.detection = None
        self.live = None
        self.solver = None
        self.revision = 0
        self.detector = Detector()
        self.frozen = False
        self.tabs = W.QTabWidget()
        self.setCentralWidget(self.tabs)
        self._setup_tab()
        self._capture_tab()
        self._result_tab()
        self.status = W.QLabel("离线就绪；没有 ROS 连接。所有结果均为 candidate，不是硬件验收。")
        self.status.setWordWrap(True)
        self.statusBar().addWidget(self.status, 1)
        self.load_config_widgets()
        self.refresh_table()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.poll)
        self.timer.start(50)

    def button(self, text, callback, layout):
        button = W.QPushButton(text)
        button.clicked.connect(lambda: self.guarded(callback))
        layout.addWidget(button)
        return button

    def guarded(self, callback):
        try:
            return callback()
        except Exception as exc:
            self.status.setText("阻止操作：" + str(exc))
            W.QMessageBox.warning(self, "检查输入 / 操作未完成", str(exc))

    def _setup_tab(self):
        page = W.QWidget()
        layout = W.QVBoxLayout(page)
        note = W.QLabel("仅支持相机、头部与 base 关系固定。桌面单图不能推断 camera-to-base。\n"
                       "DFOPTIX Q12-290-20：12×9 方格 / 11×8 内角点（88）；黑白格区域 240×180 mm；整板 290×230 mm，厚 3.2 mm。\n"
                       "原点 = 第一个内部角点，z=0 为印刷面；不自动加板厚。普通棋盘无唯一朝向，必须逐幅核对。")
        note.setWordWrap(True)
        layout.addWidget(note)
        form = W.QFormLayout()
        layout.addLayout(form)
        self.mode = W.QComboBox()
        self.mode.addItem("已知板参照：每幅独立 T_B_Q", "known_board")
        self.mode.addItem("Eye-to-hand：末端刚性装板，每幅 T_B_E", "eye_to_hand")
        form.addRow("方法", self.mode)
        self.frames = {}
        for key, text in (("base", "B / robot base"), ("color", "C / color optical"),
                          ("board", "Q / board"), ("ee", "E / end effector")):
            self.frames[key] = W.QLineEdit()
            form.addRow(text, self.frames[key])
        self.squares_x, self.squares_y = W.QSpinBox(), W.QSpinBox()
        for widget in (self.squares_x, self.squares_y):
            widget.setRange(3, 40)
        form.addRow("横向方格数（非内角点）", self.squares_x)
        form.addRow("纵向方格数（非内角点）", self.squares_y)
        self.square = W.QLineEdit()
        form.addRow("实测单格边长 / m（预填仅订单值）", self.square)
        self.measurement = W.QLineEdit()
        form.addRow("独立尺度测量记录 / 不确定度来源", self.measurement)
        self.scale_confirm = W.QCheckBox("已独立测量多格间距并确认米制尺度（不是订单尺寸）")
        form.addRow(self.scale_confirm)
        self.fixed = W.QCheckBox("确认此 session 相机/头部/base 始终固定；eye-to-hand 板与 E 刚性连接")
        form.addRow(self.fixed)
        self.raw_confirm = W.QCheckBox("确认原始未裁剪/缩放 color，K,D 是 pinhole plumb_bob；不是 rectified/inverse Brown")
        form.addRow(self.raw_confirm)
        self.profile_record = W.QLineEdit()
        form.addRow("设备序列号 / 安装与相机 profile 记录", self.profile_record)
        self.thresholds = {}
        for key, label in (("translation_mm", "closure 平移上限 / mm"), ("rotation_deg", "closure 旋转上限 / deg"),
                           ("reprojection_px", "PnP 与链预测 reprojection RMS 上限 / px")):
            self.thresholds[key] = W.QLineEdit()
            self.thresholds[key].setPlaceholderText("必须由使用方明确；无默认通过值")
            form.addRow(label, self.thresholds[key])
        self.threshold_record = W.QLineEdit()
        form.addRow("预先确定的阈值 / train-heldout 协议记录", self.threshold_record)
        self.threshold_confirm = W.QCheckBox("阈值已由使用方确认；样本数下限不代表精度验收")
        form.addRow(self.threshold_confirm)
        row = W.QHBoxLayout()
        layout.addLayout(row)
        self.new_button = self.button("新建（使用当前设置）", self.new_session, row)
        self.open_button = self.button("打开 session", self.open_session, row)
        self.save_button = self.button("保存新目录 / 新 revision", self.save_session, row)
        layout.addStretch()
        scroll = W.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(page)
        self.tabs.addTab(scroll, "1 设置 / session")
        widgets = [self.mode, *self.frames.values(), self.squares_x, self.squares_y, self.square, self.measurement,
                   self.scale_confirm, self.fixed, self.raw_confirm, self.profile_record, *self.thresholds.values(),
                   self.threshold_record, self.threshold_confirm]
        for widget in widgets:
            signal = (widget.currentIndexChanged if isinstance(widget, W.QComboBox) else
                      widget.valueChanged if isinstance(widget, W.QSpinBox) else
                      widget.toggled if isinstance(widget, W.QCheckBox) else widget.textChanged)
            signal.connect(self.setup_edited)

    def _capture_tab(self):
        page = W.QWidget()
        layout = W.QVBoxLayout(page)
        row = W.QHBoxLayout()
        layout.addLayout(row)
        self.import_button = self.button("导入 PNG + CameraInfo JSON", self.import_dialog, row)
        self.snapshot_confirm = W.QCheckBox("diagnostic 缺省 ROI/binning：已核对 full raw frame")
        row.addWidget(self.snapshot_confirm)
        row = W.QHBoxLayout()
        layout.addLayout(row)
        self.connect_allowed = W.QCheckBox("明确授权：仅订阅已有 RGB + CameraInfo")
        row.addWidget(self.connect_allowed)
        self.connect_button = self.button("连接 ROS", self.connect_live, row)
        self.disconnect_button = self.button("断开", self.disconnect_live, row)
        row.addWidget(W.QLabel("最大年龄 / s（采集检查，非同步验收）"))
        self.max_age = W.QDoubleSpinBox()
        self.max_age.setRange(0.05, 10.0)
        self.max_age.setValue(1.0)
        row.addWidget(self.max_age)
        self.preview = W.QLabel("导入离线图像，或显式连接 ROS。无需 depth，alignment 可保持关闭。")
        self.preview.setAlignment(QtCore.Qt.AlignCenter)
        self.preview.setMinimumSize(480, 260)
        self.preview.setMaximumHeight(420)
        layout.addWidget(self.preview)
        self.preview_note = W.QLabel("检测在后台运行；只保留最新待处理帧。")
        self.preview_note.setWordWrap(True)
        layout.addWidget(self.preview_note)
        row = W.QHBoxLayout()
        layout.addLayout(row)
        self.capture_button = self.button("冻结 / 加入样本（参照不复用）", self.capture, row)
        self.resume_button = self.button("返回预览", self.resume, row)
        self.reverse_button = self.button("反转当前样本角点顺序（清除关联）", self.reverse_sample, row)
        self.table = W.QTableWidget(0, 7)
        self.table.setHorizontalHeaderLabels(["ID", "split", "角点", "PnP RMS px", "方向/候选", "外部参照", "剔除原因"])
        self.table.setSelectionBehavior(W.QAbstractItemView.SelectRows)
        self.table.setSelectionMode(W.QAbstractItemView.SingleSelection)
        self.table.setEditTriggers(W.QAbstractItemView.NoEditTriggers)
        self.table.horizontalHeader().setSectionResizeMode(W.QHeaderView.Stretch)
        self.table.setMaximumHeight(150)
        self.table.itemSelectionChanged.connect(self.select_sample)
        layout.addWidget(self.table)
        row = W.QHBoxLayout()
        layout.addLayout(row)
        self.split = W.QComboBox()
        self.split.addItems(["train", "heldout"])
        row.addWidget(self.split)
        self.candidate = W.QComboBox()
        row.addWidget(self.candidate, 1)
        self.candidate_confirm = W.QCheckBox("确认此 IPPE 候选")
        row.addWidget(self.candidate_confirm)
        self.orientation_confirm = W.QCheckBox("物理确认 corner0 / +x / +y / 印刷面")
        row.addWidget(self.orientation_confirm)
        row = W.QHBoxLayout()
        layout.addLayout(row)
        self.excluded = W.QCheckBox("剔除")
        row.addWidget(self.excluded)
        self.exclusion_reason = W.QLineEdit()
        self.exclusion_reason.setPlaceholderText("剔除原因必须保留；不自动移除 outlier")
        row.addWidget(self.exclusion_reason)
        self.reference = W.QPlainTextEdit()
        self.reference.setPlaceholderText('每幅独立 JSON：{"units":"m", "from_frame":"board", "to_frame":"base",\n'
                                          '"matrix":[4×4], "source_record":"测量记录#样本", "settled_same_pose":true}\n'
                                          'eye-to-hand 的 from_frame 为 end_effector。空参照只能预览。')
        self.reference.setMaximumHeight(105)
        layout.addWidget(self.reference)
        row = W.QHBoxLayout()
        layout.addLayout(row)
        self.reference_button = self.button("读入参照 JSON 文件", self.reference_file, row)
        self.apply_button = self.button("应用当前样本（方向确认后才关联参照）", self.apply_sample, row)
        self.tabs.addTab(page, "2 预览 / 样本 / 外部参照")
        self.split.currentIndexChanged.connect(self.editor_edited)
        self.candidate.currentIndexChanged.connect(self.candidate_changed)
        for widget in (self.candidate_confirm, self.orientation_confirm, self.excluded):
            widget.toggled.connect(self.editor_edited)
        self.exclusion_reason.textChanged.connect(self.editor_edited)
        self.reference.textChanged.connect(self.editor_edited)

    def _result_tab(self):
        page = W.QWidget()
        layout = W.QVBoxLayout(page)
        label = W.QLabel("已知板：X=T_B_Q·inv(T_C_Q)，train 均值；至少 3 train + 2 heldout 独立姿态。\n"
                        "Eye-to-hand：A_i Y = X C_i，X=T_B_C，Y=T_E_Q；至少 6 train + 3 heldout，多轴旋转/平移。\n"
                        "仅 train 拟合，heldout 使用固定 X/Y。输出始终未批准，不连接机器人、不发布 TF、不改驱动。")
        label.setWordWrap(True)
        layout.addWidget(label)
        row = W.QHBoxLayout()
        layout.addLayout(row)
        self.solve_button = self.button("求解 train 并评估 heldout", self.solve_session, row)
        self.export_button = self.button("导出不可覆盖 candidate / failed diagnostic JSON", self.export_result, row)
        self.export_button.setEnabled(False)
        self.result_text = W.QPlainTextEdit()
        self.result_text.setReadOnly(True)
        layout.addWidget(self.result_text)
        self.tabs.addTab(page, "3 求解 / 留出验证 / 导出")

    def config_from_widgets(self):
        c = copy.deepcopy(self.session.config)
        c["mode"] = self.mode.currentData()
        c["frames"] = {k: w.text().strip() for k, w in self.frames.items()}
        c["board"].update(squares_x=self.squares_x.value(), squares_y=self.squares_y.value(),
                          square_m=float(self.square.text()), scale_confirmed=self.scale_confirm.isChecked(),
                          measurement_record=self.measurement.text().strip())
        c.update(fixed_camera=self.fixed.isChecked(), raw_model_confirmed=self.raw_confirm.isChecked(),
                 profile_record=self.profile_record.text().strip())
        c["thresholds"] = {k: float(w.text()) if w.text().strip() else None for k, w in self.thresholds.items()}
        c["thresholds"].update(confirmed=self.threshold_confirm.isChecked(), record=self.threshold_record.text().strip())
        return validate_config(c)

    def load_config_widgets(self):
        self.loading = True
        c = self.session.config
        self.mode.setCurrentIndex(self.mode.findData(c["mode"]))
        for k, widget in self.frames.items():
            widget.setText(c["frames"][k])
        self.squares_x.setValue(c["board"]["squares_x"])
        self.squares_y.setValue(c["board"]["squares_y"])
        self.square.setText(str(c["board"]["square_m"]))
        self.measurement.setText(c["board"]["measurement_record"])
        self.scale_confirm.setChecked(c["board"]["scale_confirmed"])
        self.fixed.setChecked(c["fixed_camera"])
        self.raw_confirm.setChecked(c["raw_model_confirmed"])
        self.profile_record.setText(c["profile_record"])
        for k, widget in self.thresholds.items():
            widget.setText("" if c["thresholds"][k] is None else str(c["thresholds"][k]))
        self.threshold_confirm.setChecked(c["thresholds"]["confirmed"])
        self.threshold_record.setText(c["thresholds"]["record"])
        self.loading = False

    def invalidate_result(self):
        self.revision += 1
        self.session.invalidate()
        self.export_button.setEnabled(False)
        self.result_text.setPlainText("输入已更改 / session 已打开：结果失效，必须重新求解。")

    def setup_edited(self, *_):
        if not self.loading:
            self.invalidate_result()
            self.generation += 1
            self.packet = self.detection = None
            self.preview_note.setText("设置已变更，请重新导入/预览。已有样本的模式、frame、几何不可混改，需新建 session。")

    def editor_edited(self, *_):
        if not self.loading:
            self.editor_dirty = True
            self.invalidate_result()

    def candidate_changed(self, *_):
        if not self.loading:
            self.candidate_confirm.setChecked(False)
            self.editor_edited()

    def sync_config(self):
        self.session.configure(self.config_from_widgets())

    def check_editor(self):
        require(not self.editor_dirty, "当前样本有未应用编辑：先应用，或重新选择样本放弃编辑")

    def replace_session(self, session):
        self.disconnect_live()
        self.session = session
        self.packet = self.detection = None
        self.editor_dirty = False
        self.frozen = False
        self.load_config_widgets()
        self.refresh_table()
        self.clear_editor()
        self.preview.clear()
        self.invalidate_result()

    def confirm_replace(self):
        return not self.session.samples or W.QMessageBox.question(
            self, "替换当前 session", "未保存编辑会丢失。继续？") == W.QMessageBox.Yes

    def new_session(self):
        config = self.config_from_widgets()
        if self.confirm_replace():
            self.replace_session(Session(config))

    def open_session(self):
        path = W.QFileDialog.getExistingDirectory(self, "选择含 session.json 的目录")
        if path and self.confirm_replace():
            self.replace_session(Session.open(path))

    def save_session(self):
        self.sync_config()
        self.check_editor()
        path, _ = W.QFileDialog.getSaveFileName(self, "新 session 目录名（不能已存在）")
        if path:
            self.session.save(path)
            self.status.setText("已保存原始 PNG / CameraInfo / 角点 / 候选 / 确认 / 参照：" + path)

    def import_dialog(self):
        image, _ = W.QFileDialog.getOpenFileName(self, "原始 color 图像", "", "Images (*.png *.jpg *.bmp)")
        if not image:
            return
        metadata, _ = W.QFileDialog.getOpenFileName(self, "CameraInfo sidecar 或 snapshots.json", str(Path(image).parent), "JSON (*.json)")
        if metadata:
            self.import_paths(image, metadata)

    def import_paths(self, image, metadata):
        self.check_editor()
        self.sync_config()
        self.disconnect_live()
        packet = load_offline(image, metadata, self.snapshot_confirm.isChecked())
        self.frozen = False
        self.table.clearSelection()
        self.clear_editor()
        self.submit_packet(packet)

    def submit_packet(self, packet):
        validate_packet(packet, self.session.config)
        if self.session.samples:
            from ..calibration.extrinsics import profile
            require(profile(packet["camera_info"]) == profile(self.session.samples[0]["camera_info"]), "Profile changed: new session required")
        self.detector.submit(packet, self.session.config, self.generation)

    def connect_live(self):
        self.check_editor()
        self.sync_config()
        require(self.connect_allowed.isChecked(), "必须显式勾选 RGB 订阅授权")
        require(self.session.config["raw_model_confirmed"], "连接前确认 raw CameraInfo 语义")
        self.disconnect_live()
        from ..calibration.extrinsics_live import LiveSource
        self.live = LiveSource()
        self.live.start()
        self.frozen = False
        self.table.clearSelection()
        self.clear_editor()
        self.status.setText("仅连接已有 RGB/CameraInfo BEST_EFFORT；等待精确配对，不启动 driver。")

    def disconnect_live(self):
        self.generation += 1
        if self.live is not None:
            self.live.stop()
            self.live = None
        self.packet = self.detection = None

    def poll(self):
        if self.solver is not None and not self.solver.is_alive():
            worker, self.solver = self.solver, None
            worker.join()
            self.solve_button.setEnabled(True)
            if worker.revision != self.revision or worker.input_sha256 != digest(self.session.inputs()):
                self.status.setText("求解期间输入已改变：丢弃过期结果，请重新求解。")
            elif worker.error:
                self.result_text.setPlainText("求解阻止：" + worker.error)
                self.status.setText("求解阻止：" + worker.error)
            else:
                self.session.result = worker.result
                self.result_text.setPlainText(json.dumps(worker.result, ensure_ascii=False, indent=2))
                self.export_button.setEnabled(True)
                self.status.setText("阈值检查：" + ("通过，但未实装验收" if worker.result["threshold_pass"]
                                                    else "失败；仅可导出 failed diagnostic"))
        if self.live is not None:
            packet, error = self.live.take()
            if error:
                self.disconnect_live()
                self.status.setText(error)
            elif packet is not None and not self.frozen:
                try:
                    self.sync_config()
                    self.submit_packet(packet)
                except Exception as exc:
                    self.disconnect_live()
                    self.status.setText("连接停止：" + str(exc))
        output = self.detector.take()
        if output is not None:
            packet, detection, generation, error = output
            if generation != self.generation or self.frozen:
                return
            if error:
                self.status.setText("检测失败：" + error)
                self.packet = self.detection = None
            else:
                self.packet, self.detection = packet, detection
                self.show_image(packet["image"], detection)
                self.preview_note.setText(f"{packet['source']} | {packet['image_header']} | "
                                          f"角点 {len(detection['corners'])} | {detection['status']}")

    def show_image(self, image, detection):
        overlay = image.copy()
        corners = np.asarray(detection["corners"], dtype=np.float32).reshape(-1, 2)
        if len(corners):
            nx = self.session.config["board"]["squares_x"] - 1
            cv2.drawChessboardCorners(overlay, (nx, self.session.config["board"]["squares_y"] - 1),
                                     corners.reshape(-1, 1, 2), True)
            p = tuple(np.rint(corners[0]).astype(int))
            cv2.circle(overlay, p, 7, (0, 0, 255), 2)
            cv2.putText(overlay, "corner0", p, cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 0, 255), 2)
            for index, text, color in ((2, "+x", (0, 0, 255)), (2 * nx, "+y", (0, 200, 0))):
                q = tuple(np.rint(corners[min(index, len(corners) - 1)]).astype(int))
                cv2.arrowedLine(overlay, p, q, color, 2)
                cv2.putText(overlay, text, q, cv2.FONT_HERSHEY_SIMPLEX, .6, color, 2)
        rgb = cv2.cvtColor(overlay, cv2.COLOR_BGR2RGB)
        qimage = QtGui.QImage(rgb.data, rgb.shape[1], rgb.shape[0], rgb.strides[0], QtGui.QImage.Format_RGB888).copy()
        self.preview.setPixmap(QtGui.QPixmap.fromImage(qimage).scaled(self.preview.size(), QtCore.Qt.KeepAspectRatio,
                                                                   QtCore.Qt.SmoothTransformation))

    def capture(self):
        self.check_editor()
        self.sync_config()
        require(not self.frozen and self.packet is not None and self.detection is not None, "先获取完整检测预览，再冻结")
        age = self.max_age.value() if self.packet["source"] == "live" else None
        self.session.add(self.packet, self.detection, age)
        self.frozen = True
        self.invalidate_result()
        self.refresh_table(len(self.session.samples) - 1)
        self.status.setText("已冻结；未复用任何参照。核对角点方向与 IPPE 候选后，单独关联本幅外部位姿。")

    def resume(self):
        self.check_editor()
        self.frozen = False
        self.table.clearSelection()
        self.clear_editor()
        self.status.setText("已返回预览；离线下一幅需重新导入。")

    def selected_index(self):
        index = self.table.currentRow()
        require(index >= 0 and bool(self.table.selectedItems()), "请选择样本")
        return index

    def refresh_table(self, selected=None):
        self.table.blockSignals(True)
        self.table.setRowCount(len(self.session.samples))
        for row, s in enumerate(self.session.samples):
            metric = "—" if s["selected"] is None else f"{s['candidates'][s['selected']]['rms_px']:.4f}"
            values = [s["id"], s["split"], str(len(s["corners"])), metric,
                      "已确认" if s["orientation_confirmed"] and s["candidate_confirmed"] else "待确认 / preview",
                      "缺失" if s["reference"] is None else s["reference"]["source_record"],
                      s["exclusion_reason"] if s["excluded"] else "使用"]
            for col, value in enumerate(values):
                self.table.setItem(row, col, W.QTableWidgetItem(value))
        self.table.clearSelection()
        self.table.blockSignals(False)
        if selected is not None:
            self.table.selectRow(selected)

    def clear_editor(self):
        self.loading = True
        self.candidate.clear()
        self.candidate.addItem("请选择（不自动采用候选 0）", None)
        self.candidate_confirm.setChecked(False)
        self.orientation_confirm.setChecked(False)
        self.reference.clear()
        self.excluded.setChecked(False)
        self.exclusion_reason.clear()
        self.loading = False
        self.editor_dirty = False

    def select_sample(self):
        if not self.table.selectedItems():
            return
        s = self.session.samples[self.table.currentRow()]
        self.clear_editor()
        self.loading = True
        self.split.setCurrentText(s["split"])
        for i, c in enumerate(s["candidates"]):
            self.candidate.addItem(f"IPPE {i}: depth={c['positive_depth']} RMS={c['rms_px']} max={c['max_px']} px", i)
        self.candidate.setCurrentIndex(0 if s["selected"] is None else s["selected"] + 1)
        self.candidate_confirm.setChecked(s["candidate_confirmed"])
        self.orientation_confirm.setChecked(s["orientation_confirmed"])
        self.excluded.setChecked(s["excluded"])
        self.exclusion_reason.setText(s["exclusion_reason"])
        self.reference.setPlainText("" if s["reference"] is None else json.dumps(s["reference"], indent=2, ensure_ascii=False))
        self.loading = False
        self.frozen = True
        self.show_image(png_image(self.session.images[s["id"]]), s)
        self.preview_note.setText(f"冻结样本 {s['id']} | reverse={s['reverse']} | 原点是内角点，不是板外角；反转会清除位姿关联")

    def reverse_sample(self):
        self.check_editor()
        self.sync_config()
        index = self.selected_index()
        self.session.reverse_order(index)
        self.invalidate_result()
        self.refresh_table(index)

    def reference_file(self):
        path, _ = W.QFileDialog.getOpenFileName(self, "本幅外部位姿 JSON", "", "JSON (*.json)")
        if path:
            self.reference.setPlainText(json.dumps(read_json(path), ensure_ascii=False, indent=2))

    def apply_sample(self):
        self.sync_config()
        index = self.selected_index()
        text = self.reference.toPlainText().strip()
        ref = json.loads(text, parse_constant=lambda _: (_ for _ in ()).throw(ValueError("Nonfinite JSON"))) if text else None
        self.session.update(index, split=self.split.currentText(), excluded=self.excluded.isChecked(),
                            exclusion_reason=self.exclusion_reason.text().strip(), selected=self.candidate.currentData(),
                            candidate_confirmed=self.candidate_confirm.isChecked(),
                            orientation_confirmed=self.orientation_confirm.isChecked(), reference=ref)
        self.editor_dirty = False
        self.invalidate_result()
        self.refresh_table(index)
        self.status.setText("样本已应用；结果失效。参照不会用于下一幅。")

    def solve_session(self):
        self.sync_config()
        self.check_editor()
        require(self.solver is None, "求解仍在运行")
        self.session.freeze_splits()
        self.invalidate_result()
        self.solver = Solver(self.session, self.revision)
        self.solve_button.setEnabled(False)
        self.solver.start()
        self.status.setText("后台求解中；split 已冻结。输入改变将丢弃结果。重复使用 heldout 不构成新的独立验收。")

    def export_result(self):
        self.sync_config()
        self.check_editor()
        path, _ = W.QFileDialog.getSaveFileName(self, "新 candidate JSON（不可覆盖）", "", "JSON (*.json)")
        if path:
            self.session.export(path)
            self.status.setText("已导出未批准候选：" + path)

    def closeEvent(self, event):
        self.timer.stop()
        self.disconnect_live()
        self.detector.stop()
        if self.solver is not None:
            self.solver.join()
            self.solver = None
        event.accept()


def main(args):
    app = W.QApplication(sys.argv[:1])
    try:
        session = Session.open(args.session) if args.session else None
        window = Window(session)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    window.show()
    return app.exec_()

