#!/usr/bin/env python3

import sys
import signal
import threading
import math
import yaml
import re

import rclpy
from rclpy.node import Node

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QScrollArea, QGroupBox, QDoubleSpinBox,
    QPushButton, QCheckBox
)
from PyQt5.QtCore import Qt, QTimer
import time

from geometry_msgs.msg import Wrench
from wbc_msgs.msg import BipedControlTarget, BipedTaskStatus


# ─── quaternion helpers ───────────────────────────────────────────────────────

def quat_to_rpy(qx, qy, qz, qw):
    sinr = 2.0 * (qw * qx + qy * qz)
    cosr = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


def rpy_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


# ─── slider layout ────────────────────────────────────────────────────────────

# (field_key, unit, min, max, scale)
FOOT_SLIDERS = [
    ("pos_x",  "m",   -0.2,           0.2,          1000),
    ("pos_y",  "m",   -0.2,           0.2,          1000),
    ("pos_z",  "m",   -1.0,           0.3,          1000),
    ("roll",   "rad", -math.pi / 4,   math.pi / 4,  1000),
    ("pitch",  "rad", -math.pi / 4,   math.pi / 4,  1000),
    ("yaw",    "rad", -math.pi / 4,   math.pi / 4,  1000),
]

BODY_SLIDERS = [
    ("pos_x",  "m",   -0.2,           0.2,          1000),
    ("pos_y",  "m",   -0.2,           0.2,          1000),
    ("pos_z",  "m",    0.4,           0.8,          1000),
    ("roll",   "rad", -math.pi / 4,   math.pi / 4,  1000),
    ("pitch",  "rad", -math.pi / 4,   math.pi / 4,  1000),
    ("yaw",    "rad", -math.pi / 4,   math.pi / 4,  1000),
]

WRENCH_SLIDERS = [
    ("force_x",  "N",  -100.0, 100.0, 10),
    ("force_y",  "N",  -100.0, 100.0, 10),
    ("force_z",  "N",     0.0, 500.0, 10),
    ("torque_x", "Nm",  -50.0,  50.0, 10),
    ("torque_y", "Nm",  -50.0,  50.0, 10),
    ("torque_z", "Nm",  -50.0,  50.0, 10),
]

# initial wrench slider values at GUI startup (overrides feedback)
WRENCH_INIT = {"force_z": 115.0}

# (display name, WbcControlTarget setpoint field, WbcTaskStatus feedback field,
#  WbcTaskStatus status field, sliders)
CART_TASKS = [
    ("Left Foot",  "left_foot_setpoint",  "left_foot_feedback",  "left_foot_task_status",  FOOT_SLIDERS),
    ("Right Foot", "right_foot_setpoint", "right_foot_feedback", "right_foot_task_status", FOOT_SLIDERS),
    ("Body Pose",  "body_pose_setpoint",  "body_pose_feedback",  "body_pose_task_status",  BODY_SLIDERS),
]

# (display name, index into WbcControlTarget contacts, WbcTaskStatus wrench feedback field)
WRENCH_TASKS = [
    ("Left Contact Wrench",  0, "wrench_left_foot"),
    ("Right Contact Wrench", 1, "wrench_right_foot"),
]

JOINT_TASK_ID = "joint_position"
# Controller config providing joint_names (WBC joint order) and robot_urdf_path (joint limits)
CONTROLLER_YAML = "/root/ros2_ws/src/wbc_ros/config/hyper_2_example/hyper_2_controllers.yaml"

INTERP_DT       = 0.01   # publish period [s] — must match create_timer interval
INTERP_DURATION = 0.5    # slider-to-target transition time [s]
OFFLINE_TIMEOUT = 1.0    # seconds without WbcTaskStatus before declaring WBC offline


def _parse_urdf_limits(urdf_file):
    """Return {joint_name: (lower, upper)} from a URDF. Uses regex because some
    URDFs contain undeclared namespaced tags (e.g. drake:) that break XML parsers."""
    with open(urdf_file, "r") as f:
        text = f.read()
    limits = {}
    for m in re.finditer(r"<joint\b([^>]*)>(.*?)</joint>", text, re.DOTALL):
        name = re.search(r'name\s*=\s*"([^"]+)"', m.group(1))
        lim = re.search(r"<limit\b([^>]*)/?>", m.group(2))
        if name is None or lim is None:
            continue
        lo = re.search(r'lower\s*=\s*"([^"]+)"', lim.group(1))
        hi = re.search(r'upper\s*=\s*"([^"]+)"', lim.group(1))
        if lo and hi:
            limits[name.group(1)] = (float(lo.group(1)), float(hi.group(1)))
    return limits


def load_joint_limits(controller_yaml):
    """Return [(joint_name, lower, upper)] in WBC joint order."""
    try:
        with open(controller_yaml, "r") as f:
            params = yaml.safe_load(f)["/**"]["ros__parameters"]
        names = params["joint_names"]
    except Exception as e:
        print(f"[wbc_gui] Could not read joint names from {controller_yaml}: {e}")
        return []

    urdf_limits = {}
    try:
        urdf_limits = _parse_urdf_limits(params["robot_urdf_path"])
    except Exception as e:
        print(f"[wbc_gui] Could not read joint limits from URDF, using [-pi, pi]: {e}")

    return [(n, *urdf_limits.get(n, (-math.pi, math.pi))) for n in names]


def rbs_init_values(rbs):
    r, p, y = quat_to_rpy(
        rbs.pose.orientation.x,
        rbs.pose.orientation.y,
        rbs.pose.orientation.z,
        rbs.pose.orientation.w,
    )
    return {
        "pos_x": rbs.pose.position.x,
        "pos_y": rbs.pose.position.y,
        "pos_z": rbs.pose.position.z,
        "roll":  r,
        "pitch": p,
        "yaw":   y,
    }


def wrench_init_values(w):
    return {
        "force_x":  w.force.x,
        "force_y":  w.force.y,
        "force_z":  w.force.z,
        "torque_x": w.torque.x,
        "torque_y": w.torque.y,
        "torque_z": w.torque.z,
    }


# ─── interpolator ─────────────────────────────────────────────────────────────

class Interpolator:
    """Scalar minimum-jerk interpolator. Produces pos, vel, acc."""

    def __init__(self, init_value=0.0):
        self.T  = INTERP_DURATION
        self.q0 = init_value
        self.q1 = init_value
        self.t  = self.T   # start "done"

    def init(self, value):
        self.q0 = value
        self.q1 = value
        self.t  = self.T

    def skip_to_target(self):
        self.t = self.T

    def set_target(self, q_target):
        pos, _, _ = self.state()
        self.q0 = pos
        self.q1 = q_target
        self.t  = 0.0

    def step(self, dt):
        self.t = min(self.t + dt, self.T)

    def state(self):
        if self.t >= self.T or self.T <= 0.0:
            return self.q1, 0.0, 0.0
        tau = self.t / self.T
        dq  = self.q1 - self.q0
        pos = self.q0 + dq * (10*tau**3 - 15*tau**4 + 6*tau**5)
        vel = dq / self.T    * (30*tau**2 - 60*tau**3  + 30*tau**4)
        acc = dq / self.T**2 * (60*tau    - 180*tau**2 + 120*tau**3)
        return pos, vel, acc


# ─── ROS node ─────────────────────────────────────────────────────────────────

class WbcGuiNode(Node):

    _POS_AXES = [("pos_x", "x"), ("pos_y", "y"), ("pos_z", "z")]
    _ROT_KEYS = ("roll", "pitch", "yaw")
    _WRENCH_FIELDS = [
        ("force_x",  "force",  "x"), ("force_y",  "force",  "y"), ("force_z",  "force",  "z"),
        ("torque_x", "torque", "x"), ("torque_y", "torque", "y"), ("torque_z", "torque", "z"),
    ]

    def __init__(self):
        super().__init__("wbc_gui")

        self.task_status_received = False
        self.task_status = None
        self.last_status_time: float = None
        self.gui_ready = False

        self.msg = BipedControlTarget()
        self.msg.contacts.active = [True, True]
        self.msg.contacts.wrenches = [Wrench(), Wrench()]

        # cart_interpolators[sp_field][field_key] = Interpolator
        self.cart_interpolators: dict = {}
        # wrench_interpolators[contact_idx][field_key] = Interpolator
        self.wrench_interpolators: dict = {}
        self.joint_interpolators: list = []

        # task IDs (sp_field or JOINT_TASK_ID) with continuous sending enabled
        self.keep_sending_tasks: set = set()
        # publish until all interpolators have converged, then stop
        self.send_until_done: bool = False

        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, depth=1
        )

        self.pub = self.create_publisher(BipedControlTarget, "whole_body_control/setpoint", qos)
        self.sub = self.create_subscription(
            BipedTaskStatus, "whole_body_control/task_status", self._task_status_cb, qos
        )
        self.timer = self.create_timer(INTERP_DT, self._publish)

    def _task_status_cb(self, msg):
        self.task_status = msg
        self.task_status_received = True
        self.last_status_time = time.time()

    # ── message building ───────────────────────────────────────────────────────

    def _write_msg(self, step=False):
        for sp_field, field_interps in self.cart_interpolators.items():
            rbs = getattr(self.msg, sp_field)

            for interp in field_interps.values():
                if step:
                    interp.step(INTERP_DT)

            for field_key, axis in self._POS_AXES:
                if field_key in field_interps:
                    pos, vel, acc = field_interps[field_key].state()
                    setattr(rbs.pose.position, axis, pos)
                    setattr(rbs.twist.linear, axis, vel)
                    setattr(rbs.acceleration.linear, axis, acc)

            if all(k in field_interps for k in self._ROT_KEYS):
                r,  rv, ra = field_interps["roll"].state()
                p,  pv, pa = field_interps["pitch"].state()
                y,  yv, ya = field_interps["yaw"].state()
                qx, qy, qz, qw = rpy_to_quat(r, p, y)
                rbs.pose.orientation.x = qx
                rbs.pose.orientation.y = qy
                rbs.pose.orientation.z = qz
                rbs.pose.orientation.w = qw
                rbs.twist.angular.x    = rv
                rbs.twist.angular.y    = pv
                rbs.twist.angular.z    = yv
                rbs.acceleration.angular.x = ra
                rbs.acceleration.angular.y = pa
                rbs.acceleration.angular.z = ya

        for contact_idx, field_interps in self.wrench_interpolators.items():
            wrench = self.msg.contacts.wrenches[contact_idx]

            for interp in field_interps.values():
                if step:
                    interp.step(INTERP_DT)

            for field_key, vec, axis in self._WRENCH_FIELDS:
                if field_key in field_interps:
                    pos, _, _ = field_interps[field_key].state()
                    setattr(getattr(wrench, vec), axis, pos)

        for idx, interp in enumerate(self.joint_interpolators):
            if step:
                interp.step(INTERP_DT)
            pos, vel, acc = interp.state()
            self.msg.joint_position_setpoint.position[idx]     = pos
            self.msg.joint_position_setpoint.velocity[idx]     = vel
            self.msg.joint_position_setpoint.acceleration[idx] = acc

    def _publish(self):
        if not self.gui_ready:
            return
        if not self.keep_sending_tasks and not self.send_until_done:
            return

        self._write_msg(step=True)
        self.pub.publish(self.msg)

        # In send-once mode, stop as soon as every interpolator has converged
        if self.send_until_done and not self.keep_sending_tasks:
            all_done = (
                all(interp.t >= interp.T
                    for fi in self.cart_interpolators.values()
                    for interp in fi.values())
                and
                all(interp.t >= interp.T
                    for fi in self.wrench_interpolators.values()
                    for interp in fi.values())
                and
                all(interp.t >= interp.T
                    for interp in self.joint_interpolators)
            )
            if all_done:
                self.send_until_done = False

    # ── called by GUI ──────────────────────────────────────────────────────────

    def send_now(self):
        """Start publishing; stop automatically once all interpolators converge."""
        self.send_until_done = True

    def set_keep_sending(self, task_id, enabled):
        if enabled:
            self.keep_sending_tasks.add(task_id)
        else:
            self.keep_sending_tasks.discard(task_id)

    def init_rbs_field(self, sp_field, key, value):
        self.cart_interpolators.setdefault(sp_field, {})[key] = Interpolator(value)

    def set_rbs_target(self, sp_field, key, value):
        self.cart_interpolators[sp_field][key].set_target(value)

    def init_wrench_field(self, contact_idx, key, value):
        self.wrench_interpolators.setdefault(contact_idx, {})[key] = Interpolator(value)

    def set_wrench_target(self, contact_idx, key, value):
        self.wrench_interpolators[contact_idx][key].set_target(value)

    def set_contact_active(self, contact_idx, active):
        self.msg.contacts.active[contact_idx] = active

    def init_joint_positions(self, n):
        self.joint_interpolators = [Interpolator() for _ in range(n)]
        self.msg.joint_position_setpoint.position     = [0.0] * n
        self.msg.joint_position_setpoint.velocity     = [0.0] * n
        self.msg.joint_position_setpoint.acceleration = [0.0] * n

    def init_joint(self, idx, value):
        if idx < len(self.joint_interpolators):
            self.joint_interpolators[idx].init(value)

    def set_joint_target(self, idx, value):
        if idx < len(self.joint_interpolators):
            self.joint_interpolators[idx].set_target(value)


# ─── GUI ──────────────────────────────────────────────────────────────────────

class WbcGui(QWidget):
    def __init__(self, node, joint_limits):
        super().__init__()
        self.node = node
        self.setWindowTitle("WBC GUI")
        self.resize(800, 800)

        # Wait for first task_status (up to 3 s)
        t0 = time.time()
        while not node.task_status_received and time.time() - t0 < 3.0:
            QApplication.processEvents()
            time.sleep(0.02)

        if not node.task_status_received:
            node.get_logger().warn("No WbcTaskStatus message received — GUI will be disabled. Is the debug flag set in the WBC node?")

        container = QWidget()
        layout = QVBoxLayout(container)
        self.indicators = {}
        self._cart_slider_widgets = {}      # (sp_field, field_key) -> (slider, spinbox, scale, lo, hi)
        self._wrench_slider_widgets = {}    # (contact_idx, field_key) -> (slider, spinbox, scale, lo, hi)
        self._joint_slider_widgets = []     # list of (slider, spinbox, scale, lo, hi)
        self._keep_sending_checkboxes = []  # all "Keep Sending" QCheckBox widgets
        self._wbc_online = node.task_status_received

        # ── Cartesian tasks ────────────────────────────────────────────────────
        for display_name, sp_field, fb_field, status_field, sliders in CART_TASKS:
            if node.task_status is not None:
                init = rbs_init_values(getattr(node.task_status, fb_field))
            else:
                init = {k: 0.0 for k, *_ in sliders}

            group = QGroupBox(display_name)
            glayout = QVBoxLayout(group)

            indicator = QLabel("● unknown")
            indicator.setStyleSheet("color: gray;")
            glayout.addWidget(indicator)
            self.indicators[status_field] = indicator

            refresh_fn = lambda sp=sp_field, fb=fb_field, sl=sliders: (
                self._sync_cart_task(self.node.task_status, sp, fb, sl)
                if self.node.task_status is not None else None
            )
            self._make_send_controls(glayout, sp_field, refresh_fn)

            for key, unit, lo, hi, scale in sliders:
                init_v = max(lo, min(hi, init[key]))
                self._make_slider_row(glayout, key, unit, lo, hi, scale,
                                      init_v, sp_field, key)

            layout.addWidget(group)

        # ── Contact wrench tasks ───────────────────────────────────────────────
        for display_name, contact_idx, fb_field in WRENCH_TASKS:
            if node.task_status is not None:
                init = wrench_init_values(getattr(node.task_status, fb_field))
            else:
                init = {k: 0.0 for k, *_ in WRENCH_SLIDERS}
            init.update(WRENCH_INIT)

            group = QGroupBox(display_name)
            glayout = QVBoxLayout(group)

            refresh_fn = lambda ci=contact_idx, fb=fb_field: (
                self._sync_wrench_task(self.node.task_status, ci, fb)
                if self.node.task_status is not None else None
            )
            self._make_send_controls(glayout, f"contact_wrench_{contact_idx}", refresh_fn)

            active_cb = QCheckBox("Contact Active")
            active_cb.stateChanged.connect(
                lambda state, ci=contact_idx: self.node.set_contact_active(ci, bool(state))
            )
            active_cb.setChecked(True)
            glayout.addWidget(active_cb)

            for key, unit, lo, hi, scale in WRENCH_SLIDERS:
                init_v = max(lo, min(hi, init[key]))
                self._make_wrench_row(glayout, key, unit, lo, hi, scale,
                                      init_v, contact_idx)

            layout.addWidget(group)

        # ── Joint positions ────────────────────────────────────────────────────
        if joint_limits:
            fb_positions = []
            if node.task_status is not None:
                fb_positions = list(node.task_status.joint_position_feedback.position)

            node.init_joint_positions(len(joint_limits))
            joint_group = QGroupBox("Joint Positions")
            jl = QVBoxLayout(joint_group)
            joint_indicator = QLabel("● unknown")
            joint_indicator.setStyleSheet("color: gray;")
            jl.addWidget(joint_indicator)
            self.indicators["joint_position_task_status"] = joint_indicator

            joint_refresh_fn = lambda: (
                self._sync_joint_task(self.node.task_status)
                if self.node.task_status is not None else None
            )
            self._make_send_controls(jl, JOINT_TASK_ID, joint_refresh_fn)

            for idx, (name, lo, hi) in enumerate(joint_limits):
                init_v = fb_positions[idx] if idx < len(fb_positions) else 0.0
                init_v = max(lo, min(hi, init_v))
                self._make_joint_row(jl, name, lo, hi, 1000, init_v, idx)
            layout.addWidget(joint_group)

        # ── Scroll area ────────────────────────────────────────────────────────
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(container)

        main_layout = QVBoxLayout(self)
        main_layout.addWidget(scroll)

        self._indicator_timer = QTimer(self)
        self._indicator_timer.timeout.connect(self._update_indicators)
        self._indicator_timer.start(200)

        node.gui_ready = True

        if not node.task_status_received:
            self.setEnabled(False)

    # ── helpers ────────────────────────────────────────────────────────────────

    def _make_send_controls(self, parent_layout, task_id, refresh_fn):
        row = QHBoxLayout()

        send_btn = QPushButton("Send")
        send_btn.setFixedWidth(70)
        send_btn.clicked.connect(self.node.send_now)

        refresh_btn = QPushButton("Update")
        refresh_btn.setFixedWidth(70)
        refresh_btn.clicked.connect(lambda: refresh_fn())

        keep_cb = QCheckBox("Keep Sending")

        def on_keep_sending(state, btn=send_btn, tid=task_id):
            enabled = bool(state)
            self.node.set_keep_sending(tid, enabled)
            btn.setEnabled(not enabled)

        keep_cb.stateChanged.connect(on_keep_sending)
        self._keep_sending_checkboxes.append(keep_cb)

        row.addWidget(send_btn)
        row.addWidget(keep_cb)
        row.addStretch()
        row.addWidget(refresh_btn)
        parent_layout.addLayout(row)

    def _make_slider_row(self, parent_layout, key, unit, lo, hi, scale,
                         init_v, sp_field, field_key):
        lo_i = int(lo * scale)
        hi_i = int(hi * scale)
        init_i = max(lo_i, min(hi_i, int(init_v * scale)))

        label = QLabel(f"{key} ({unit})" if unit else key)
        label.setFixedWidth(120)

        slider = QSlider(Qt.Horizontal)
        slider.setMinimum(lo_i)
        slider.setMaximum(hi_i)
        slider.setValue(init_i)

        spinbox = QDoubleSpinBox()
        spinbox.setMinimum(lo)
        spinbox.setMaximum(hi)
        spinbox.setSingleStep(1.0 / scale)
        spinbox.setDecimals(3)
        spinbox.setValue(init_v)
        spinbox.setFixedWidth(90)

        def on_slider(v, sb=spinbox, sf=sp_field, fk=field_key):
            phys = v / scale
            sb.blockSignals(True)
            sb.setValue(phys)
            sb.blockSignals(False)
            self.node.set_rbs_target(sf, fk, phys)

        def on_spinbox(v, sl=slider, sf=sp_field, fk=field_key):
            sl.blockSignals(True)
            sl.setValue(int(v * scale))
            sl.blockSignals(False)
            self.node.set_rbs_target(sf, fk, v)

        slider.valueChanged.connect(on_slider)
        spinbox.valueChanged.connect(on_spinbox)

        row = QHBoxLayout()
        row.addWidget(label)
        row.addWidget(slider)
        row.addWidget(spinbox)
        parent_layout.addLayout(row)
        self.node.init_rbs_field(sp_field, field_key, init_v)
        self._cart_slider_widgets[(sp_field, field_key)] = (slider, spinbox, scale, lo, hi)

    def _make_wrench_row(self, parent_layout, key, unit, lo, hi, scale,
                         init_v, contact_idx):
        lo_i = int(lo * scale)
        hi_i = int(hi * scale)
        init_i = max(lo_i, min(hi_i, int(init_v * scale)))

        label = QLabel(f"{key} ({unit})" if unit else key)
        label.setFixedWidth(120)

        slider = QSlider(Qt.Horizontal)
        slider.setMinimum(lo_i)
        slider.setMaximum(hi_i)
        slider.setValue(init_i)

        spinbox = QDoubleSpinBox()
        spinbox.setMinimum(lo)
        spinbox.setMaximum(hi)
        spinbox.setSingleStep(1.0 / scale)
        spinbox.setDecimals(3)
        spinbox.setValue(init_v)
        spinbox.setFixedWidth(90)

        def on_slider(v, sb=spinbox, ci=contact_idx, fk=key):
            phys = v / scale
            sb.blockSignals(True)
            sb.setValue(phys)
            sb.blockSignals(False)
            self.node.set_wrench_target(ci, fk, phys)

        def on_spinbox(v, sl=slider, ci=contact_idx, fk=key):
            sl.blockSignals(True)
            sl.setValue(int(v * scale))
            sl.blockSignals(False)
            self.node.set_wrench_target(ci, fk, v)

        slider.valueChanged.connect(on_slider)
        spinbox.valueChanged.connect(on_spinbox)

        row = QHBoxLayout()
        row.addWidget(label)
        row.addWidget(slider)
        row.addWidget(spinbox)
        parent_layout.addLayout(row)
        self.node.init_wrench_field(contact_idx, key, init_v)
        self._wrench_slider_widgets[(contact_idx, key)] = (slider, spinbox, scale, lo, hi)

    def _make_joint_row(self, parent_layout, name, lo, hi, scale, init_v, idx):
        lo_i, hi_i = int(lo * scale), int(hi * scale)
        init_i = max(lo_i, min(hi_i, int(init_v * scale)))

        label = QLabel(name)
        label.setFixedWidth(120)

        slider = QSlider(Qt.Horizontal)
        slider.setMinimum(lo_i)
        slider.setMaximum(hi_i)
        slider.setValue(init_i)

        spinbox = QDoubleSpinBox()
        spinbox.setMinimum(lo)
        spinbox.setMaximum(hi)
        spinbox.setSingleStep(1.0 / scale)
        spinbox.setDecimals(3)
        spinbox.setValue(init_v)
        spinbox.setFixedWidth(90)

        def on_slider(v, sb=spinbox, i=idx):
            phys = v / scale
            sb.blockSignals(True)
            sb.setValue(phys)
            sb.blockSignals(False)
            self.node.set_joint_target(i, phys)

        def on_spinbox(v, sl=slider, i=idx):
            sl.blockSignals(True)
            sl.setValue(int(v * scale))
            sl.blockSignals(False)
            self.node.set_joint_target(i, v)

        slider.valueChanged.connect(on_slider)
        spinbox.valueChanged.connect(on_spinbox)

        row = QHBoxLayout()
        row.addWidget(label)
        row.addWidget(slider)
        row.addWidget(spinbox)
        parent_layout.addLayout(row)
        self.node.init_joint(idx, init_v)
        self._joint_slider_widgets.append((slider, spinbox, scale, lo, hi))

    def _sync_cart_task(self, ts, sp_field, fb_field, sliders):
        init = rbs_init_values(getattr(ts, fb_field))
        for key, _unit, lo, hi, _scale in sliders:
            entry = self._cart_slider_widgets.get((sp_field, key))
            if entry is None:
                continue
            slider, spinbox, scale, lo, hi = entry
            v = max(lo, min(hi, init[key]))
            slider.blockSignals(True)
            spinbox.blockSignals(True)
            slider.setValue(int(v * scale))
            spinbox.setValue(v)
            slider.blockSignals(False)
            spinbox.blockSignals(False)
            self.node.init_rbs_field(sp_field, key, v)

    def _sync_wrench_task(self, ts, contact_idx, fb_field):
        init = wrench_init_values(getattr(ts, fb_field))
        for key, _unit, lo, hi, _scale in WRENCH_SLIDERS:
            entry = self._wrench_slider_widgets.get((contact_idx, key))
            if entry is None:
                continue
            slider, spinbox, scale, lo, hi = entry
            v = max(lo, min(hi, init[key]))
            slider.blockSignals(True)
            spinbox.blockSignals(True)
            slider.setValue(int(v * scale))
            spinbox.setValue(v)
            slider.blockSignals(False)
            spinbox.blockSignals(False)
            self.node.init_wrench_field(contact_idx, key, v)

    def _sync_joint_task(self, ts):
        fb_positions = list(ts.joint_position_feedback.position)
        for idx, (slider, spinbox, scale, lo, hi) in enumerate(self._joint_slider_widgets):
            v = fb_positions[idx] if idx < len(fb_positions) else 0.0
            v = max(lo, min(hi, v))
            slider.blockSignals(True)
            spinbox.blockSignals(True)
            slider.setValue(int(v * scale))
            spinbox.setValue(v)
            slider.blockSignals(False)
            spinbox.blockSignals(False)
            self.node.init_joint(idx, v)

    def _sync_sliders_from_status(self, ts):
        for _, sp_field, fb_field, _, sliders in CART_TASKS:
            self._sync_cart_task(ts, sp_field, fb_field, sliders)
        for _, contact_idx, fb_field in WRENCH_TASKS:
            self._sync_wrench_task(ts, contact_idx, fb_field)
        self._sync_joint_task(ts)

    def _update_indicators(self):
        wbc_online = (
            self.node.last_status_time is not None
            and time.time() - self.node.last_status_time < OFFLINE_TIMEOUT
        )

        if not wbc_online and self._wbc_online:
            # WBC just went offline: mark indicators, uncheck keep-sending, disable GUI
            for indicator in self.indicators.values():
                indicator.setStyleSheet("color: gray;")
                indicator.setText("● offline")
            for cb in self._keep_sending_checkboxes:
                cb.setChecked(False)   # fires stateChanged → clears keep_sending_tasks
            self.setEnabled(False)
            self._wbc_online = False

        elif wbc_online and not self._wbc_online:
            # WBC just came back online: sync sliders from latest status, re-enable GUI
            self._sync_sliders_from_status(self.node.task_status)
            self.setEnabled(True)
            self._wbc_online = True

        if not wbc_online:
            return

        ts = self.node.task_status
        if ts is None:
            return
        for status_field, indicator in self.indicators.items():
            activation = getattr(ts, status_field).activation
            if activation >= 0.5:
                indicator.setStyleSheet("color: green;")
                indicator.setText("● active")
            else:
                indicator.setStyleSheet("color: red;")
                indicator.setText("● inactive")


# ─── entry point ──────────────────────────────────────────────────────────────

def ros_spin(node):
    rclpy.spin(node)


def main():
    rclpy.init()

    joint_limits = load_joint_limits(CONTROLLER_YAML)

    node = WbcGuiNode()

    ros_thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    ros_thread.start()

    app = QApplication(sys.argv)

    signal.signal(signal.SIGINT, lambda *_: app.quit())
    sigint_timer = QTimer()
    sigint_timer.start(200)
    sigint_timer.timeout.connect(lambda: None)

    gui = WbcGui(node, joint_limits)
    gui.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
