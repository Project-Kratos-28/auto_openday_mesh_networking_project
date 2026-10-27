#!/usr/bin/env python3
import sys
import json
import math
import time
import zenoh
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QGroupBox, QLabel, QFrame, QSplitter, QPushButton,
    QDoubleSpinBox, QSlider, QTextEdit
)
from PyQt6.QtCore import QThread, QTimer, pyqtSignal, Qt
from PyQt6.QtGui import QFont, QImage, QPixmap

import pyqtgraph as pg

# Configure global PyQtGraph style
pg.setConfigOption('background', '#121212')
pg.setConfigOption('foreground', '#E0E0E0')


def open_zenoh_session() -> zenoh.Session:
    """Use IPv4 explicitly so IPv4-only machines can still join the local mesh."""
    config = zenoh.Config()
    config.insert_json5("listen/endpoints", '["tcp/0.0.0.0:0"]')
    return zenoh.open(config)


# ------------------------------------------------------------------------------
# 1. Zenoh Listener Background Thread
# ------------------------------------------------------------------------------
class ZenohListenerThread(QThread):
    odom_signal = pyqtSignal(dict)
    event_signal = pyqtSignal(dict)
    error_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.session = None
        self._subscribers = []

    def run(self):
        try:
            self.session = open_zenoh_session()
            # Keep explicit handles to make subscriber lifetime deterministic.
            self._subscribers = [
                self.session.declare_subscriber(
                    "mesh/positions", lambda sample: self._emit_json(sample, self.odom_signal)
                ),
                self.session.declare_subscriber(
                    "mesh/base/telemetry", lambda sample: self._emit_json(sample, self.odom_signal)
                ),
                self.session.declare_subscriber(
                    "mesh/failure/events", lambda sample: self._emit_json(sample, self.event_signal)
                ),
            ]
            self.exec()
        except Exception as exc:
            self.error_signal.emit(f"Zenoh listener failed: {exc}")
        finally:
            for subscriber in self._subscribers:
                try:
                    subscriber.undeclare()
                except Exception:
                    pass
            self._subscribers.clear()
            if self.session is not None:
                try:
                    self.session.close()
                except Exception:
                    pass
                self.session = None

    @staticmethod
    def _emit_json(sample, signal):
        try:
            data = json.loads(sample.payload.to_string())
            if isinstance(data, dict):
                signal.emit(data)
        except Exception as exc:
            print(f"[Zenoh Thread] Error parsing payload: {exc}")

    def stop(self):
        if self.session is not None:
            try:
                self.session.close()
            except Exception:
                pass
        self.quit()
        if self.isRunning():
            self.wait(2000)


# ------------------------------------------------------------------------------
# 2. Camera Feed Sub-Window Widget (Rover Local & Base Received Feeds)
# ------------------------------------------------------------------------------
class CameraFeedWidget(QGroupBox):
    def __init__(self, title: str, subtitle: str):
        super().__init__(title)
        self.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 1px solid #333333;
                border-radius: 6px;
                margin-top: 6px;
                background-color: #1A1A1A;
                color: #00E5FF;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
        """)

        layout = QVBoxLayout(self)

        self.status_label = QLabel(subtitle)
        self.status_label.setStyleSheet("color: #888888; font-size: 11px;")
        layout.addWidget(self.status_label)

        self.video_label = QLabel("Waiting for Camera Stream...")
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video_label.setStyleSheet("""
            QLabel {
                background-color: #050505;
                border: 1px dashed #444444;
                color: #555555;
                font-size: 13px;
            }
        """)
        self.video_label.setMinimumSize(300, 200)
        layout.addWidget(self.video_label, stretch=1)

    def update_frame(self, q_image: QImage, fps: float = 0.0, drop_rate: float = 0.0):
        pixmap = QPixmap.fromImage(q_image)
        scaled = pixmap.scaled(self.video_label.size(), Qt.AspectRatioMode.KeepAspectRatio)
        self.video_label.setPixmap(scaled)
        self.status_label.setText(f"FPS: {fps:.1f} | Packet Loss: {drop_rate:.1f}%")

    def show_placeholder(self, message: str):
        self.video_label.setText(message)


# ------------------------------------------------------------------------------
# 3. Sub-Window 1: Odometry & Trajectory Map Widget
# ------------------------------------------------------------------------------
class OdometryWidget(QGroupBox):
    waypoint_picked = pyqtSignal(float, float)

    def __init__(self):
        super().__init__("Sub-Window 1: Rover Odometry & Trajectory Map")
        self.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 1px solid #333333;
                border-radius: 6px;
                margin-top: 6px;
                background-color: #1A1A1A;
                color: #00FF66;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
        """)

        layout = QVBoxLayout(self)

        # Telemetry Text Cards Grid
        grid = QGridLayout()
        self.lbl_x = self._create_card("X Position", "0.00 m")
        self.lbl_y = self._create_card("Y Position", "0.00 m")
        self.lbl_z = self._create_card("Z Position", "0.00 m")
        self.lbl_yaw = self._create_card("Yaw Angle", "0.0°")
        self.lbl_speed = self._create_card("Linear Speed", "0.00 m/s")
        self.lbl_hops = self._create_card("Active Route", "Direct")

        grid.addWidget(self.lbl_x['box'], 0, 0)
        grid.addWidget(self.lbl_y['box'], 0, 1)
        grid.addWidget(self.lbl_z['box'], 0, 2)
        grid.addWidget(self.lbl_yaw['box'], 1, 0)
        grid.addWidget(self.lbl_speed['box'], 1, 1)
        grid.addWidget(self.lbl_hops['box'], 1, 2)

        layout.addLayout(grid)

        # 2D PyQtGraph Map
        self.plot_widget = pg.PlotWidget(title="Live Path Trail (Double-Click Map to Pick Waypoint)")
        self.plot_widget.setLabel('bottom', "X Coordinates (m)")
        self.plot_widget.setLabel('left', "Y Coordinates (m)")
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setAspectLocked(True)

        self.path_curve = self.plot_widget.plot(pen=pg.mkPen(color='#00FF66', width=2))
        self.rover_marker = self.plot_widget.plot(
            pen=None, symbol='o', symbolSize=12, symbolBrush=pg.mkBrush('#FF3366')
        )

        # Double-click listener for map clicking
        self.plot_widget.scene().sigMouseClicked.connect(self._on_map_clicked)
        layout.addWidget(self.plot_widget, stretch=1)

        self.x_history = []
        self.y_history = []
        self.last_time = None
        self.last_pos = None

    def _create_card(self, title: str, default_val: str) -> dict:
        box = QFrame()
        box.setStyleSheet("background-color: #242424; border-radius: 4px; padding: 4px;")
        l = QVBoxLayout(box)
        l.setContentsMargins(4, 4, 4, 4)

        t_lbl = QLabel(title)
        t_lbl.setStyleSheet("color: #888888; font-size: 10px; font-weight: normal;")
        v_lbl = QLabel(default_val)
        v_lbl.setStyleSheet("color: #FFFFFF; font-size: 13px; font-weight: bold;")

        l.addWidget(t_lbl)
        l.addWidget(v_lbl)
        return {'box': box, 'val': v_lbl}

    def process_odometry(self, data: dict):
        if not isinstance(data, dict):
            return
        payload = data.get("payload") if isinstance(data.get("payload"), dict) else {}

        def read_number(key):
            value = data.get(key, payload.get(key))
            if value is None:
                return None
            try:
                number = float(value)
            except (TypeError, ValueError):
                return None
            return number if math.isfinite(number) else None

        x, y, z = (read_number("x"), read_number("y"), read_number("z"))
        # Ignore unrelated/malformed packets rather than injecting false (0, 0, 0) points.
        if x is None or y is None:
            nested_position = payload.get("position")
            if isinstance(nested_position, dict):
                try:
                    x = float(nested_position["x"])
                    y = float(nested_position["y"])
                    z = float(nested_position.get("z", 0.0))
                except (KeyError, TypeError, ValueError):
                    return
            else:
                return
        if z is None:
            z = 0.0
        if not all(math.isfinite(value) for value in (x, y, z)):
            return

        yaw = read_number("yaw")
        reported_speed = read_number("linear_speed")
        if reported_speed is None:
            reported_speed = read_number("speed")

        route = data.get("path", data.get("route", payload.get("path", payload.get("route"))))
        if isinstance(route, (list, tuple)) and all(isinstance(hop, str) for hop in route):
            self.lbl_hops['val'].setText(" → ".join(route) if route else "Direct")

        curr_time = time.monotonic()
        estimated_speed = 0.0
        if self.last_pos is not None and self.last_time is not None:
            dx = x - self.last_pos[0]
            dy = y - self.last_pos[1]
            dt = curr_time - self.last_time
            distance = math.hypot(dx, dy)
            # Ignore teleports or gaps; otherwise a single stale packet can create a speed spike.
            if 0.02 <= dt <= 2.0 and distance <= 5.0:
                estimated_speed = distance / dt
            if yaw is None and distance > 0.01:
                yaw = math.atan2(dy, dx)

        speed = reported_speed if reported_speed is not None else estimated_speed
        if not math.isfinite(speed) or speed < 0:
            speed = 0.0

        self.lbl_x['val'].setText(f"{x:+.2f} m")
        self.lbl_y['val'].setText(f"{y:+.2f} m")
        self.lbl_z['val'].setText(f"{z:+.2f} m")
        self.lbl_speed['val'].setText(f"{speed:.2f} m/s")
        if yaw is not None and math.isfinite(yaw):
            self.lbl_yaw['val'].setText(f"{math.degrees(yaw):.1f}°")

        self.last_pos = (x, y)
        self.last_time = curr_time
        self.x_history.append(x)
        self.y_history.append(y)
        # Bound history and plotting costs during long simulation runs.
        if len(self.x_history) > 5000:
            del self.x_history[:1000]
            del self.y_history[:1000]
        self.path_curve.setData(self.x_history, self.y_history)
        self.rover_marker.setData([x], [y])

    def _on_map_clicked(self, event):
        if event.double():
            pos = event.scenePos()
            view_box = self.plot_widget.plotItem.vb
            if not view_box.sceneBoundingRect().contains(pos):
                return
            mouse_point = view_box.mapSceneToView(pos)
            if math.isfinite(mouse_point.x()) and math.isfinite(mouse_point.y()):
                self.waypoint_picked.emit(mouse_point.x(), mouse_point.y())


# ------------------------------------------------------------------------------
# 4. Sub-Window 4: Flight & Motion Control (E-STOP, cmd_vel, Waypoints)
# ------------------------------------------------------------------------------
class ControlPanelWidget(QGroupBox):
    def __init__(self, zenoh_session: zenoh.Session):
        super().__init__("Sub-Window 4: Flight & Motion Control")
        self.session = zenoh_session
        self.estop_active = False
        self._manual_command = (0.0, 0.0)
        self._command_heartbeat = QTimer(self)
        self._command_heartbeat.setInterval(200)
        self._command_heartbeat.timeout.connect(self._repeat_manual_command)
        self._command_heartbeat.start()

        self.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 1px solid #333333;
                border-radius: 6px;
                margin-top: 6px;
                background-color: #1A1A1A;
                color: #FFCC00;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
        """)

        layout = QVBoxLayout(self)

        # Emergency Stop Button
        self.btn_estop = QPushButton("EMERGENCY STOP (E-STOP)")
        self.btn_estop.setFont(QFont("Arial", 13, QFont.Weight.Bold))
        self.btn_estop.setMinimumHeight(45)
        self.btn_estop.setStyleSheet("""
            QPushButton {
                background-color: #D32F2F;
                color: white;
                border: 2px solid #FF5252;
                border-radius: 6px;
            }
            QPushButton:hover { background-color: #F44336; }
            QPushButton:pressed { background-color: #B71C1C; }
        """)
        self.btn_estop.clicked.connect(self.toggle_estop)
        layout.addWidget(self.btn_estop)

        self.estop_status_label = QLabel("SYSTEM STATUS: NORMAL OPERATION")
        self.estop_status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.estop_status_label.setStyleSheet("color: #00FF66; font-weight: bold; font-size: 11px;")
        layout.addWidget(self.estop_status_label)

        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.HLine)
        sep.setStyleSheet("color: #333;")
        layout.addWidget(sep)

        # Manual Directional Commands (cmd_vel)
        layout.addWidget(QLabel("<b>Manual Velocity Control (cmd_vel)</b><br>Commands latch; press Stop or E-STOP to halt."))
        cmd_grid = QGridLayout()

        self.btn_up = QPushButton("▲ Forward")
        self.btn_left = QPushButton("◄ Turn Left")
        self.btn_right = QPushButton("Turn Right ►")
        self.btn_down = QPushButton("▼ Reverse")
        self.btn_stop = QPushButton("■ Stop")

        self.btn_up.clicked.connect(lambda: self.send_cmd_vel(1.0, 0.0))
        self.btn_down.clicked.connect(lambda: self.send_cmd_vel(-1.0, 0.0))
        self.btn_left.clicked.connect(lambda: self.send_cmd_vel(0.0, 0.8))
        self.btn_right.clicked.connect(lambda: self.send_cmd_vel(0.0, -0.8))
        self.btn_stop.clicked.connect(lambda: self.send_cmd_vel(0.0, 0.0))

        cmd_grid.addWidget(self.btn_up, 0, 1)
        cmd_grid.addWidget(self.btn_left, 1, 0)
        cmd_grid.addWidget(self.btn_stop, 1, 1)
        cmd_grid.addWidget(self.btn_right, 1, 2)
        cmd_grid.addWidget(self.btn_down, 2, 1)
        layout.addLayout(cmd_grid)

        # Velocity Sliders
        slider_layout = QHBoxLayout()
        slider_layout.addWidget(QLabel("Linear:"))
        self.slider_linear = QSlider(Qt.Orientation.Horizontal)
        self.slider_linear.setRange(-20, 20)
        self.slider_linear.setValue(0)
        self.slider_linear.valueChanged.connect(self._on_slider_changed)
        slider_layout.addWidget(self.slider_linear)

        slider_layout.addWidget(QLabel("Angular:"))
        self.slider_angular = QSlider(Qt.Orientation.Horizontal)
        self.slider_angular.setRange(-15, 15)
        self.slider_angular.setValue(0)
        self.slider_angular.valueChanged.connect(self._on_slider_changed)
        slider_layout.addWidget(self.slider_angular)
        layout.addLayout(slider_layout)

        sep2 = QFrame()
        sep2.setFrameShape(QFrame.Shape.HLine)
        sep2.setStyleSheet("color: #333;")
        layout.addWidget(sep2)

        # Waypoint Dispatcher
        layout.addWidget(QLabel("<b>Waypoint Target Dispatcher</b>"))
        wp_layout = QHBoxLayout()

        wp_layout.addWidget(QLabel("Target X:"))
        self.spin_x = QDoubleSpinBox()
        self.spin_x.setRange(-235.0, 235.0)
        self.spin_x.setValue(0.0)
        wp_layout.addWidget(self.spin_x)

        wp_layout.addWidget(QLabel("Target Y:"))
        self.spin_y = QDoubleSpinBox()
        self.spin_y.setRange(-235.0, 235.0)
        self.spin_y.setValue(0.0)
        wp_layout.addWidget(self.spin_y)

        self.btn_send_wp = QPushButton("Send Waypoint")
        self.btn_send_wp.setStyleSheet("background-color: #00897B; color: white; font-weight: bold;")
        self.btn_send_wp.clicked.connect(self.send_waypoint)
        wp_layout.addWidget(self.btn_send_wp)

        layout.addLayout(wp_layout)

    def toggle_estop(self):
        self.estop_active = not self.estop_active
        if self.estop_active:
            self.btn_estop.setText("RESET EMERGENCY STOP")
            self.btn_estop.setStyleSheet("QPushButton { background-color: #FF9800; color: black; font-weight: bold; }")
            self.estop_status_label.setText("⚠️ EMERGENCY STOP ACTIVATED — ROVER LOCKED ⚠️")
            self.estop_status_label.setStyleSheet("color: #FF3366; font-weight: bold; font-size: 11px;")

            estop_pkt = {
                "type": "ESTOP",
                "src": "base",
                "dst": "rover",
                "sent_time": time.time(),
                "payload": {"estop": True}
            }
            self._manual_command = (0.0, 0.0)
            self.slider_linear.blockSignals(True)
            self.slider_angular.blockSignals(True)
            self.slider_linear.setValue(0)
            self.slider_angular.setValue(0)
            self.slider_linear.blockSignals(False)
            self.slider_angular.blockSignals(False)
            self._publish_packet("mesh/base/cmd", estop_pkt)
        else:
            self.btn_estop.setText("EMERGENCY STOP (E-STOP)")
            self.btn_estop.setStyleSheet("QPushButton { background-color: #D32F2F; color: white; font-weight: bold; }")
            self.estop_status_label.setText("SYSTEM STATUS: NORMAL OPERATION")
            self.estop_status_label.setStyleSheet("color: #00FF66; font-weight: bold; font-size: 11px;")

            clear_pkt = {
                "type": "ESTOP",
                "src": "base",
                "dst": "rover",
                "sent_time": time.time(),
                "payload": {"estop": False}
            }
            self._manual_command = (0.0, 0.0)
            self.slider_linear.blockSignals(True)
            self.slider_angular.blockSignals(True)
            self.slider_linear.setValue(0)
            self.slider_angular.setValue(0)
            self.slider_linear.blockSignals(False)
            self.slider_angular.blockSignals(False)
            self._publish_packet("mesh/base/cmd", clear_pkt)

    def _publish_packet(self, key: str, packet: dict):
        try:
            self.session.put(key, json.dumps(packet, allow_nan=False))
        except Exception as exc:
            print(f"[ControlPanel] Could not publish {key}: {exc}")

    def _publish_command(self, linear_x: float, angular_z: float):
        cmd_pkt = {
            "type": "COMMAND",
            "src": "base",
            "dst": "rover",
            "sent_time": time.time(),
            "payload": {"linear_x": linear_x, "angular_z": angular_z}
        }
        self._publish_packet("mesh/base/cmd", cmd_pkt)

    def send_cmd_vel(self, linear_x: float, angular_z: float, override_estop: bool = False):
        if self.estop_active and not override_estop:
            print("[ControlPanel] Motion blocked: E-STOP Active!")
            return
        try:
            linear_x = float(linear_x)
            angular_z = float(angular_z)
            if not math.isfinite(linear_x) or not math.isfinite(angular_z):
                raise ValueError("velocity must be finite")
        except (TypeError, ValueError) as exc:
            print(f"[ControlPanel] Invalid velocity command: {exc}")
            return
        self._manual_command = (max(-2.0, min(2.0, linear_x)), max(-1.5, min(1.5, angular_z)))
        self._publish_command(*self._manual_command)

    def _repeat_manual_command(self):
        if self.estop_active:
            return
        linear_x, angular_z = self._manual_command
        if linear_x != 0.0 or angular_z != 0.0:
            self._publish_command(linear_x, angular_z)

    def _on_slider_changed(self):
        linear = self.slider_linear.value() / 10.0
        angular = self.slider_angular.value() / 10.0
        self.send_cmd_vel(linear, angular)

    def send_waypoint(self):
        if self.estop_active:
            print("[ControlPanel] Waypoint blocked: E-STOP Active!")
            return

        # Stop the latched manual-command heartbeat before dispatching a waypoint,
        # otherwise the next heartbeat would cancel waypoint mode immediately.
        self._manual_command = (0.0, 0.0)
        self.slider_linear.blockSignals(True)
        self.slider_angular.blockSignals(True)
        self.slider_linear.setValue(0)
        self.slider_angular.setValue(0)
        self.slider_linear.blockSignals(False)
        self.slider_angular.blockSignals(False)

        wp_pkt = {
            "type": "WAYPOINT",
            "src": "base",
            "dst": "rover",
            "sent_time": time.time(),
            "payload": {
                "target_x": self.spin_x.value(),
                "target_y": self.spin_y.value(),
                "max_speed": 1.0
            }
        }
        self._publish_packet("mesh/base/cmd", wp_pkt)
        print(f"[ControlPanel] Dispatched Waypoint -> ({self.spin_x.value()}, {self.spin_y.value()})")

    def set_waypoint_coords(self, x: float, y: float):
        self.spin_x.setValue(x)
        self.spin_y.setValue(y)


# ------------------------------------------------------------------------------
# 5. Sub-Window 5: Failure Injection Controller & Event Console
# ------------------------------------------------------------------------------
class FailureControlWidget(QGroupBox):
    def __init__(self, zenoh_session: zenoh.Session):
        super().__init__("Sub-Window 5: Failure Injector & Event Log")
        self.session = zenoh_session

        self.setStyleSheet("""
            QGroupBox {
                font-weight: bold;
                border: 1px solid #333333;
                border-radius: 6px;
                margin-top: 6px;
                background-color: #1A1A1A;
                color: #FF5252;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
        """)

        layout = QVBoxLayout(self)

        # The failure-injection consumer is still a planned package. Make this
        # explicit instead of presenting a slider that has no effect.
        self.failure_status = QLabel(
            "Failure injection is inactive until the failure_injection node is implemented."
        )
        self.failure_status.setWordWrap(True)
        self.failure_status.setStyleSheet("color: #FFCC00; font-size: 11px;")
        layout.addWidget(self.failure_status)
        self.slider_drop = QSlider(Qt.Orientation.Horizontal)
        self.slider_drop.setRange(0, 100)
        self.slider_drop.setEnabled(False)
        self.slider_drop.setToolTip("Inactive: no failure-injection consumer exists yet.")
        layout.addWidget(self.slider_drop)

        # Event Console
        layout.addWidget(QLabel("<b>Network Failure Event Stream:</b>"))
        self.log_console = QTextEdit()
        self.log_console.setReadOnly(True)
        self.log_console.setStyleSheet("background-color: #080808; color: #FF5252; font-family: monospace;")
        layout.addWidget(self.log_console, stretch=1)

    def _update_failures(self):
        val = self.slider_drop.value() / 100.0
        msg = {"drop_probability": val}
        self.session.put("mesh/failure/control", json.dumps(msg))

    def log_event(self, data: dict):
        self.log_console.append(f"[EVENT] {json.dumps(data)}")


# ------------------------------------------------------------------------------
# 6. Main Dashboard Window Layout
# ------------------------------------------------------------------------------
class BaseStationMainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Mars Rover Relay Mesh — Base Station Ground Control Console")
        self.resize(1380, 850)
        self.setStyleSheet("background-color: #0F0F0F;")

        # Direct Zenoh Session for GUI Publishing
        self.zenoh_session = open_zenoh_session()

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # Dashboard Header
        header = QLabel("MARS ROVER RELAY MESH — GROUND STATION CONTROL CONSOLE")
        header.setStyleSheet("color: #00E5FF; font-size: 16px; font-weight: bold; padding: 4px;")
        main_layout.addWidget(header)

        # Splitter dividing Left (Odometry + Control) and Right (Cameras + Failure Injection)
        main_splitter = QSplitter(Qt.Orientation.Horizontal)

        # LEFT COLUMN
        left_container = QWidget()
        left_layout = QVBoxLayout(left_container)
        left_layout.setContentsMargins(0, 0, 0, 0)

        self.odom_widget = OdometryWidget()
        self.control_widget = ControlPanelWidget(self.zenoh_session)

        # Connect map double-click picking to waypoint spinboxes
        self.odom_widget.waypoint_picked.connect(self.control_widget.set_waypoint_coords)

        left_layout.addWidget(self.odom_widget, stretch=2)
        left_layout.addWidget(self.control_widget, stretch=1)
        main_splitter.addWidget(left_container)

        # RIGHT COLUMN
        right_container = QWidget()
        right_layout = QVBoxLayout(right_container)
        right_layout.setContentsMargins(0, 0, 0, 0)

        # Dual Camera Feeds
        self.rover_cam_widget = CameraFeedWidget(
            "Sub-Window 2: Rover Camera (Direct Local Stream)",
            "Status: Ready for ROS 2 Image Bridge"
        )
        self.rover_cam_widget.show_placeholder("Sub-Window 2\n[ Direct Camera Feed Reserved ]")

        self.base_cam_widget = CameraFeedWidget(
            "Sub-Window 3: Base Station Camera (Mesh Delivered Stream)",
            "Status: Ready for Zenoh Mesh JPEG Packet Decoder"
        )
        self.base_cam_widget.show_placeholder("Sub-Window 3\n[ Received Mesh Video Reserved ]")

        self.failure_widget = FailureControlWidget(self.zenoh_session)

        cam_splitter = QSplitter(Qt.Orientation.Horizontal)
        cam_splitter.addWidget(self.rover_cam_widget)
        cam_splitter.addWidget(self.base_cam_widget)

        right_layout.addWidget(cam_splitter, stretch=1)
        right_layout.addWidget(self.failure_widget, stretch=1)

        main_splitter.addWidget(right_container)

        # Balance left and right view splits
        main_splitter.setSizes([650, 730])
        main_layout.addWidget(main_splitter)

        # Start Zenoh Listener Thread
        self.listener_thread = ZenohListenerThread()
        self.listener_thread.odom_signal.connect(self.odom_widget.process_odometry)
        self.listener_thread.event_signal.connect(self.failure_widget.log_event)
        self.listener_thread.error_signal.connect(self.statusBar().showMessage)
        self.listener_thread.start()

    def closeEvent(self, event):
        self.listener_thread.stop()
        try:
            self.zenoh_session.close()
        except Exception as exc:
            print(f"[BaseStation] Error closing Zenoh session: {exc}")
        event.accept()


# ------------------------------------------------------------------------------
# Entry Point
# ------------------------------------------------------------------------------
def main():
    app = QApplication(sys.argv)
    window = BaseStationMainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
