"""Base station GUI - PLACEHOLDER framework (PyQt6). Run: ros2 run base_station_gui gui

Working now:  the mesh ON/OFF toggle + live status of the mesh, base A and base B.
Placeholders: dashed boxes marked TODO, for the GUI teammate to fill in.
All mesh data comes from MeshBridge (mesh_bridge.py), so widgets never touch Zenoh.
"""
import sys
import time

from PyQt6.QtCore import QRectF, QSize, Qt, QTimer
from PyQt6.QtGui import QColor, QPainter
from PyQt6.QtWidgets import (QAbstractButton, QApplication, QFrame, QGridLayout, QGroupBox,
                             QHBoxLayout, QLabel, QMainWindow, QVBoxLayout, QWidget)

from base_station_gui.mesh_bridge import MeshBridge

REFRESH_MS = 250              # redraw 4 times a second
CONFIRM_TIMEOUT = 4.0         # s to wait for the coordinator to confirm a toggle
GREEN, RED, AMBER, GREY = "#2e9e4f", "#c8402f", "#d08a1c", "#808080"


class ToggleSwitch(QAbstractButton):
    """A sliding ON/OFF switch (a checkable button that paints itself as a pill)."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setCheckable(True)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setFixedSize(QSize(64, 32))

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        on = self.isChecked()
        track = QColor(GREEN if on else "#9a9a9a")
        if not self.isEnabled():
            track = QColor("#cfcfcf")
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(track)
        p.drawRoundedRect(QRectF(0, 0, self.width(), self.height()), 16, 16)
        p.setBrush(QColor("white"))
        x = self.width() - 29 if on else 3
        p.drawEllipse(QRectF(x, 3, 26, 26))


def status_label(text="-"):
    lab = QLabel(text)
    lab.setTextFormat(Qt.TextFormat.RichText)
    lab.setWordWrap(True)
    return lab


def dot(color, text):
    return f'<span style="color:{color}">&#9679;</span> <b>{text}</b>'


def placeholder(title, what, key):
    """A dashed box saying what goes here and which data to use."""
    box = QFrame()
    box.setStyleSheet("QFrame { border: 2px dashed #b0b0b0; border-radius: 8px; }"
                      "QLabel { border: none; color: #707070; }")
    lay = QVBoxLayout(box)
    t = QLabel(f"<b>TODO: {title}</b>")
    t.setAlignment(Qt.AlignmentFlag.AlignCenter)
    d = QLabel(f"{what}<br><small>data: <code>{key}</code></small>")
    d.setAlignment(Qt.AlignmentFlag.AlignCenter)
    d.setWordWrap(True)
    lay.addStretch()
    lay.addWidget(t)
    lay.addWidget(d)
    lay.addStretch()
    box.setMinimumHeight(140)
    return box


class BaseStationWindow(QMainWindow):
    def __init__(self, bridge):
        super().__init__()
        self.bridge = bridge
        self.pending = None                    # (wanted_on, time) while waiting for confirmation
        self.setWindowTitle("Base Station - Mars Relay Mesh")
        self.resize(1000, 680)

        # ---- top bar: the mesh toggle
        self.toggle = ToggleSwitch()
        self.toggle.clicked.connect(self.on_toggle)
        self.toggle_text = QLabel("<b>Mesh network</b>")
        self.toggle_state = status_label()
        top = QHBoxLayout()
        top.addWidget(self.toggle_text)
        top.addWidget(self.toggle)
        top.addWidget(self.toggle_state, 1)

        # ---- status cards
        self.mesh_info, self.a_info, self.b_info = status_label(), status_label(), status_label()
        cards = QHBoxLayout()
        for title, lab in (("Mesh / coordinator", self.mesh_info),
                           ("Base A - commander", self.a_info),
                           ("Base B - odometry receiver", self.b_info)):
            g = QGroupBox(title)
            QVBoxLayout(g).addWidget(lab)
            lab.setAlignment(Qt.AlignmentFlag.AlignTop)
            cards.addWidget(g, 1)

        # ---- placeholders for the GUI teammate
        grid = QGridLayout()
        grid.addWidget(placeholder("Map / link view", "Top-down map: units, crater, live links",
                                   "snapshot()['positions'], ['links']"), 0, 0)
        grid.addWidget(placeholder("Packet route animation", "Animate the route of each packet",
                                   "snapshot()['base_b_last_odom']['route']"), 0, 1)
        grid.addWidget(placeholder("Rover telemetry plots", "Odometry / speed / battery over time",
                                   "snapshot()['base_b_last_odom']['payload']"), 1, 0)
        grid.addWidget(placeholder("Failure injection controls", "Sliders: corrupt / drop / delay",
                                   "publish to mesh/failure/control"), 1, 1)

        central = QWidget()
        lay = QVBoxLayout(central)
        lay.addLayout(top)
        lay.addLayout(cards)
        lay.addLayout(grid, 1)
        self.setCentralWidget(central)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(REFRESH_MS)
        self.refresh()

    # ---------------------------------------------------------- toggle
    def on_toggle(self, checked):
        """User clicked the switch: ask the coordinator, then wait for it to confirm."""
        self.bridge.set_mesh(checked)
        self.pending = (checked, time.time())
        self.refresh()

    # ---------------------------------------------------------- redraw
    def refresh(self):
        s = self.bridge.snapshot()
        mode = s["mesh_mode"]
        if self.pending:                                    # waiting for confirmation
            wanted, t0 = self.pending
            if mode == ("on" if wanted else "off"):
                self.pending = None
            elif time.time() - t0 > CONFIRM_TIMEOUT:
                self.pending = None
                self.toggle_state.setText(dot(RED, "coordinator did not confirm - is it running?"))
        if not s["coordinator_running"]:
            self.toggle.setEnabled(False)
            self.toggle_state.setText(dot(GREY, "coordinator not running "
                                                "(start: ros2 launch mesh_protocol mesh.launch.py)"))
        else:
            self.toggle.setEnabled(True)
            if self.pending:
                self.toggle.setChecked(self.pending[0])
                self.toggle_state.setText(dot(AMBER, f"switching {'ON' if self.pending[0] else 'OFF'} ..."))
            else:
                self.toggle.setChecked(mode == "on")        # follow the real state (CLI can change it)
                text = {"on": dot(GREEN, "ON - drones relaying"), "off": dot(GREY, "OFF - direct links only"),
                        "auto": dot(AMBER, "AUTO - deploys if base A loses the rover")}.get(mode, mode)
                self.toggle_state.setText(text)

        # mesh card
        if s["coordinator_running"]:
            ch = s["chains"] or {}
            lines = [f"State: <b>{s['mesh_state']}</b>", f"<small>{s['mesh_reason']}</small>"]
            for b in ("base_a", "base_b"):
                if b in ch:
                    lines.append(f"{b}: rover &rarr; {' &rarr; '.join(ch[b]) or '(direct)'} &rarr; {b}")
            m = s["mission"]
            if m:
                lines.append(f"Mission: {m['state']} - x {m['x']} m ({m['progress']} %)")
            self.mesh_info.setText("<br>".join(lines))
        else:
            self.mesh_info.setText(dot(GREY, "no data"))

        # base A card
        a = s["base_a_last_ack"]
        if s["base_a_connected"] and a:
            self.a_info.setText("<br>".join([
                dot(GREEN, "CONNECTED"), f"round trip: {a['rtt_ms']} ms",
                f"command: {' &rarr; '.join(a['cmd_route'] or [])}",
                f"ack: {' &rarr; '.join(a['ack_route'])}", f"acks received: {s['base_a_acks']}"]))
        else:
            self.a_info.setText(dot(RED, "NO CONNECTION") + "<br>no answer from the rover")

        # base B card
        o = s["base_b_last_odom"]
        if s["base_b_receiving"] and o:
            p = o["payload"]
            od, tr = p.get("odom"), p.get("truth")
            lines = [dot(GREEN, "RECEIVING ODOMETRY"), f"route: {' &rarr; '.join(o['route'])}",
                     f"delay: {o['delay_ms']} ms   msgs: {s['base_b_msgs']}"]
            if od:
                lines.append(f"odom: x {od['x']}  y {od['y']}  v {od['v']} m/s")
            if tr:
                lines.append(f"true position: {tr['pos']}")
            gap = s["odom_longest_gap_since_on"]
            if gap is not None:
                lines.append(f"longest gap since mesh ON: {gap:.1f} s")
            self.b_info.setText("<br>".join(lines))
        else:
            self.b_info.setText(dot(RED, "NO DATA") + "<br>rover not reachable")

    def closeEvent(self, event):
        self.timer.stop()
        self.bridge.close()
        super().closeEvent(event)


def main():
    app = QApplication(sys.argv)
    win = BaseStationWindow(MeshBridge())
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
