#!/usr/bin/env python3
import sys
import subprocess
import signal
import os
import json
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
    QLabel, QMessageBox, QGridLayout, QTextEdit, QFrame, QSlider,
    QGroupBox
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
import threading

# ========================================================================
# CONFIG PATH (inside workspace source tree)
# ========================================================================

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


_PKG_SRC_DIR = _SCRIPT_DIR
while os.path.basename(_PKG_SRC_DIR) != 'darput_description' and _PKG_SRC_DIR != '/':
    _PKG_SRC_DIR = os.path.dirname(_PKG_SRC_DIR)

CONFIG_DIR = os.path.join(_PKG_SRC_DIR, 'config')
CONFIG_PATH = os.path.join(CONFIG_DIR, 'ui_config.json')

# ========================================================================
# COMMANDS CONFIGURATION
# Format tombol : ("btn", "Label", "shell command")
# Format slider : ("slider", "Label", "topic", "msg_type", min, max, default, step)
# ========================================================================
COMMANDS = [
    ("btn", "Pre Walking", "ros2 run darput_description Pre_Walking_state"),
    ("btn", "Walking", "ros2 run darput_description Walking_State"),
    ("btn", "Initial Position", "ros2 run darput_description Init_Position"),
    ("btn", "Visual Marker", "ros2 run darput_description VisualMarker"),
    ("btn", "Reset IK Memory", "ros2 service call /reset_ik_memory std_srvs/srv/Trigger"),
    ("btn", "Reset Simulation Pos", 'ign service -s /world/empty/set_pose --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean --timeout 2000 --req \'name: "darput", position: {x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}\''),
    ("btn", "Go walk", 'ros2 topic pub --once /start_walking std_msgs/msg/Int64 "data: 1"'),
    ("btn", "Colcon Build", "colcon build"),

    # FORMAT: ("slider", label, topic, msg_type, min_val, max_val, default_val, step)
    ("slider", "Target Pitch (in degrees)", "/target_pitch", "Int64", -60, 60, 0, 1),
]


# ========================================================================
# ROS 2 PUBLISHER NODE (Background Thread)
# ========================================================================
class RosPublisherNode(Node):
    def __init__(self):
        super().__init__('ui_publisher_node')
        self._pubs = {}

    def ensure_publisher(self, topic, msg_type):
        if topic not in self._pubs:
            if msg_type == "Int64":
                self._pubs[topic] = self.create_publisher(Int64, topic, 10)
            else:
                self.get_logger().warn(f"Unsupported msg_type '{msg_type}' for {topic}")
                return None
            self.get_logger().info(f"Created publisher: {topic} ({msg_type})")
        return self._pubs[topic]

    def publish_value(self, topic, msg_type, value):
        pub = self.ensure_publisher(topic, msg_type)
        if pub is None:
            return
        if msg_type == "Int64":
            msg = Int64()
            msg.data = int(value)
            pub.publish(msg)


# ========================================================================
# MAIN UI CLASS
# ========================================================================
class SimpleLauncher(QWidget):
    def __init__(self):
        super().__init__()
        self.processes = []
        self.ros_node = None
        self.ros_executor = None
        self.ros_thread = None

        self.slider_widgets = {}      # topic -> QSlider
        self.slider_value_labels = {} # topic -> QLabel

        self._init_ros()
        self._init_ui()
        self._setup_signals()
        self._load_config()

    # ------------------------------------------------------------------
    # ROS 2 Lifecycle
    # ------------------------------------------------------------------
    def _init_ros(self):
        if not rclpy.ok():
            rclpy.init(args=None)
        self.ros_node = RosPublisherNode()
        self.ros_executor = rclpy.executors.MultiThreadedExecutor()
        self.ros_executor.add_node(self.ros_node)
        self.ros_thread = threading.Thread(target=self.ros_executor.spin, daemon=True)
        self.ros_thread.start()

    def _cleanup_ros(self):
        if self.ros_executor is not None:
            self.ros_executor.shutdown()
            self.ros_executor = None
        if self.ros_node is not None:
            self.ros_node.destroy_node()
            self.ros_node = None
        if rclpy.ok():
            rclpy.shutdown()

    # ------------------------------------------------------------------
    # Signals & Close Event
    # ------------------------------------------------------------------
    def _setup_signals(self):
        signal.signal(signal.SIGINT, self.handle_interrupt)
        signal.signal(signal.SIGTERM, self.handle_interrupt)
        self._sig_timer = QTimer(self)
        self._sig_timer.timeout.connect(lambda: None)
        self._sig_timer.start(100)

    def handle_interrupt(self, signum, frame):
        self.log("Interrupt received, shutting down...")
        self.kill_all_processes()
        self._cleanup_ros()
        QApplication.quit()

    def closeEvent(self, event):
        reply = QMessageBox.question(
            self, 'Exit', 'Exit and stop all processes?\n(Unsaved slider changes will be LOST unless you pressed Save Config)',
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.kill_all_processes()
            self._cleanup_ros()
            event.accept()
        else:
            event.ignore()

    # ------------------------------------------------------------------
    # UI Build
    # ------------------------------------------------------------------
    def _init_ui(self):
        self.setWindowTitle('Darput ROS2 Launcher')
        self.setMinimumSize(700, 500)
        self.resize(800, 600)

        main = QVBoxLayout()
        main.setSpacing(15)
        main.setContentsMargins(20, 20, 20, 20)

        # Title
        title = QLabel("Darput Command Launcher")
        title.setFont(QFont("Arial", 16, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        main.addWidget(title)

        # ---- Button Grid ----
        btn_group = QGroupBox("Commands")
        btn_grid = QGridLayout()
        btn_grid.setSpacing(10)
        btn_row = 0
        btn_col = 0

        for cmd in COMMANDS:
            if cmd[0] != "btn":
                continue
            label, shell_cmd = cmd[1], cmd[2]
            btn = QPushButton(label)
            btn.setMinimumHeight(40)
            btn.clicked.connect(lambda checked, c=shell_cmd, l=label: self.run_command(c, l))
            btn_grid.addWidget(btn, btn_row, btn_col)
            btn_col += 1
            if btn_col >= 2:
                btn_col = 0
                btn_row += 1

        btn_group.setLayout(btn_grid)
        main.addWidget(btn_group)

        # ---- Slider Area ----
        slider_group = QGroupBox("Live Parameter Sliders")
        slider_layout = QVBoxLayout()
        slider_layout.setSpacing(12)

        for cmd in COMMANDS:
            if cmd[0] != "slider":
                continue
            _, label_text, topic, msg_type, min_v, max_v, default_v, step = cmd
            self._create_slider_row(slider_layout, label_text, topic, msg_type,
                                    min_v, max_v, default_v, step)

        slider_group.setLayout(slider_layout)
        main.addWidget(slider_group)

        # ---- Log Area ----
        log_label = QLabel("Output Log:")
        log_label.setFont(QFont("Arial", 10, QFont.Bold))
        main.addWidget(log_label)

        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(150)
        main.addWidget(self.log_text)

        # ---- Bottom Controls ----
        bottom = QHBoxLayout()
        self.status = QLabel("Ready")
        bottom.addWidget(self.status)
        bottom.addStretch()

        save_btn = QPushButton("Save Config")
        save_btn.setStyleSheet("background-color: #4ecdc4; color: white; font-weight: bold;")
        save_btn.setToolTip(f"Save current slider values & window layout to:\n{CONFIG_PATH}")
        save_btn.clicked.connect(self._save_config)
        bottom.addWidget(save_btn)

        stop_btn = QPushButton("Stop All")
        stop_btn.setStyleSheet("background-color: #ff6b6b; color: white; font-weight: bold;")
        stop_btn.clicked.connect(self.kill_all_processes)
        bottom.addWidget(stop_btn)

        clear_btn = QPushButton("Clear Log")
        clear_btn.clicked.connect(self.log_text.clear)
        bottom.addWidget(clear_btn)

        main.addLayout(bottom)
        self.setLayout(main)

    def _create_slider_row(self, parent_layout, label_text, topic, msg_type,
                           min_v, max_v, default_v, step):
        row = QHBoxLayout()
        row.setSpacing(10)

        label = QLabel(f"{label_text}:")
        label.setMinimumWidth(140)
        label.setFont(QFont("Arial", 9, QFont.Bold))

        slider = QSlider(Qt.Horizontal)
        slider.setMinimum(min_v)
        slider.setMaximum(max_v)
        slider.setValue(default_v)
        slider.setSingleStep(step)
        slider.setPageStep(step * 5)

        value_label = QLabel(str(default_v))
        value_label.setMinimumWidth(40)
        value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        value_label.setStyleSheet("color: #0066cc; font-weight: bold;")

        self.slider_widgets[topic] = slider
        self.slider_value_labels[topic] = value_label

        slider.valueChanged.connect(
            lambda val, t=topic, m=msg_type, lbl=value_label:
            self._on_slider_changed(t, m, val, lbl)
        )

        row.addWidget(label)
        row.addWidget(slider, 1)
        row.addWidget(value_label)
        parent_layout.addLayout(row)

    # ------------------------------------------------------------------
    # Slider Logic (publish only, no auto-save)
    # ------------------------------------------------------------------
    def _on_slider_changed(self, topic, msg_type, value, value_label):
        value_label.setText(str(value))
        if self.ros_node is not None:
            self.ros_node.publish_value(topic, msg_type, value)

    # ------------------------------------------------------------------
    # Config Save / Load (Manual)
    # ------------------------------------------------------------------
    def _save_config(self):
        """Manual save: only runs when Save Config button is pressed."""
        config = {
            "geometry": {
                "x": self.pos().x(),
                "y": self.pos().y(),
                "width": self.width(),
                "height": self.height()
            },
            "sliders": {}
        }
        for topic, slider in self.slider_widgets.items():
            config["sliders"][topic] = slider.value()
        try:
            os.makedirs(CONFIG_DIR, exist_ok=True)
            with open(CONFIG_PATH, 'w') as f:
                json.dump(config, f, indent=2)
            self.log(f"💾 Config saved: {CONFIG_PATH}")
        except Exception as e:
            self.log(f"❌ Save config error: {e}")

    def _load_config(self):
        """Load config at startup. If file doesn't exist, use defaults."""
        if not os.path.exists(CONFIG_PATH):
            self.log("ℹ️ No saved config found. Using default slider values.")
            return
        try:
            with open(CONFIG_PATH, 'r') as f:
                config = json.load(f)

            # Window geometry
            geo = config.get("geometry", {})
            if "x" in geo and "y" in geo:
                self.move(geo["x"], geo["y"])
            if "width" in geo and "height" in geo:
                self.resize(geo["width"], geo["height"])

            # Slider values (block signal to avoid publish storm on startup)
            for topic, val in config.get("sliders", {}).items():
                if topic in self.slider_widgets:
                    slider = self.slider_widgets[topic]
                    slider.blockSignals(True)
                    slider.setValue(val)
                    slider.blockSignals(False)
                    if topic in self.slider_value_labels:
                        self.slider_value_labels[topic].setText(str(val))

            self.log(f"📂 Config loaded from: {CONFIG_PATH}")
        except Exception as e:
            self.log(f"❌ Load config error: {e}")

    # ------------------------------------------------------------------
    # Process Runner (untuk tombol)
    # ------------------------------------------------------------------
    def run_command(self, cmd, label):
        self.log(f"Starting: {label}")
        self.log(f"  Command: {cmd}")
        try:
            proc = subprocess.Popen(
                cmd,
                shell=True,
                executable='/bin/bash',
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )
            self.processes.append(proc)
            self.update_status()
            self.log(f"  PID: {proc.pid}")
        except Exception as e:
            self.log(f"ERROR: {e}")

    def kill_all_processes(self):
        if not self.processes:
            self.log("No active processes")
            return
        self.log("Stopping all processes...")
        for proc in list(self.processes):
            try:
                if proc.poll() is None:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                    proc.wait(timeout=2)
            except:
                pass
        self.processes = []
        self.update_status()
        self.log("All processes stopped")

    def update_status(self):
        active = sum(1 for p in self.processes if p.poll() is None)
        self.status.setText(f"Active: {active}")

    def log(self, msg):
        self.log_text.append(msg)
        scrollbar = self.log_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())


# ========================================================================
# ENTRY POINT
# ========================================================================
def main(args=None):
    app = QApplication(sys.argv)
    launcher = SimpleLauncher()
    launcher.show()
    try:
        sys.exit(app.exec())
    except AttributeError:
        sys.exit(app.exec_())


if __name__ == '__main__':
    main()
