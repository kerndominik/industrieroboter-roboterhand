"""Control tab: finger sliders, grasp presets, demo sequence, servo power."""

import math

from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import (
    QCheckBox, QDoubleSpinBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QProgressBar,
    QPushButton, QSlider, QVBoxLayout, QWidget)

from .ros_node import FINGERS

THUMB_ROTATION_JOINT = 'thumb_rotation_joint'
THUMB_ROTATION_RANGE_DEG = (-30.0, 105.0)
THUMB_ROTATION_CAD_DEG = -16.4

# name: (closure thumb, index, middle, ring, little), thumb rotation [deg]
PRESETS = {
    'Open hand': ([0.0, 0.0, 0.0, 0.0, 0.0], THUMB_ROTATION_CAD_DEG),
    'Fist': ([0.8, 1.0, 1.0, 1.0, 1.0], 60.0),
    'Pinch': ([0.6, 0.6, 0.0, 0.0, 0.0], 90.0),
    'Tripod': ([0.6, 0.55, 0.55, 0.0, 0.0], 90.0),
    'Cylinder grasp': ([0.5, 0.6, 0.6, 0.6, 0.6], 75.0),
    'Point': ([0.8, 0.0, 1.0, 1.0, 1.0], 60.0),
    'Peace': ([0.8, 0.0, 0.0, 1.0, 1.0], 60.0),
    'Thumbs up': ([0.0, 1.0, 1.0, 1.0, 1.0], THUMB_ROTATION_CAD_DEG),
}
DEMO_SEQUENCE = ['Open hand', 'Fist', 'Open hand', 'Pinch', 'Tripod', 'Cylinder grasp',
                 'Open hand', 'Point', 'Peace', 'Thumbs up', 'Open hand']


class ControlTab(QWidget):

    def __init__(self, node, parent=None):
        super().__init__(parent)
        self.node = node
        self.dirty = False
        self.sequence = None
        self.synced = False  # sliders take over the actual pose once at startup

        layout = QHBoxLayout(self)
        layout.addWidget(self._build_finger_box(), 3)
        right = QVBoxLayout()
        right.addWidget(self._build_preset_box())
        right.addWidget(self._build_sequence_box())
        right.addWidget(self._build_hardware_box())
        right.addStretch(1)
        layout.addLayout(right, 2)

        self.send_timer = QTimer(self)
        self.send_timer.timeout.connect(self._send_if_dirty)
        self.send_timer.start(40)  # max. 25 commands/s
        self.seq_timer = QTimer(self)
        self.seq_timer.timeout.connect(self._sequence_step)

    # ------------------------------------------------------------- layout
    def _build_finger_box(self):
        box = QGroupBox('Finger closure (0 % = open, 100 % = closed)')
        grid = QGridLayout(box)
        for col, title in enumerate(['Finger', 'Command', '', 'Actual', 'MCP angle']):
            lbl = QLabel(f'<b>{title}</b>')
            grid.addWidget(lbl, 0, col)
        self.sliders, self.spins, self.bars, self.angle_labels = [], [], [], []
        for row, finger in enumerate(FINGERS, start=1):
            grid.addWidget(QLabel(finger.capitalize()), row, 0)
            slider = QSlider(Qt.Horizontal)
            slider.setRange(0, 1000)
            spin = QDoubleSpinBox()
            spin.setRange(0.0, 100.0)
            spin.setDecimals(1)
            spin.setSuffix(' %')
            slider.valueChanged.connect(lambda v, s=spin: self._slider_changed(v, s))
            spin.valueChanged.connect(lambda v, s=slider: self._spin_changed(v, s))
            bar = QProgressBar()
            bar.setRange(0, 1000)
            bar.setFormat('%p %')
            angle = QLabel('-')
            angle.setMinimumWidth(70)
            grid.addWidget(slider, row, 1)
            grid.addWidget(spin, row, 2)
            grid.addWidget(bar, row, 3)
            grid.addWidget(angle, row, 4)
            self.sliders.append(slider)
            self.spins.append(spin)
            self.bars.append(bar)
            self.angle_labels.append(angle)

        row = len(FINGERS) + 1
        grid.addWidget(QLabel('Thumb rotation\n(passive joint)'), row, 0)
        self.thumb_slider = QSlider(Qt.Horizontal)
        lo, hi = THUMB_ROTATION_RANGE_DEG
        self.thumb_slider.setRange(int(lo * 10), int(hi * 10))
        self.thumb_spin = QDoubleSpinBox()
        self.thumb_spin.setRange(lo, hi)
        self.thumb_spin.setDecimals(1)
        self.thumb_spin.setSuffix(' deg')
        self.thumb_slider.valueChanged.connect(
            lambda v: self._slider_changed(v, self.thumb_spin, scale=10.0))
        self.thumb_spin.valueChanged.connect(
            lambda v: self._spin_changed(v, self.thumb_slider, scale=10.0))
        self.thumb_actual = QLabel('-')
        grid.addWidget(self.thumb_slider, row, 1)
        grid.addWidget(self.thumb_spin, row, 2)
        grid.addWidget(self.thumb_actual, row, 4)
        self._set_thumb_rotation(THUMB_ROTATION_CAD_DEG)

        row += 1
        self.live_check = QCheckBox('Send continuously while moving the sliders')
        self.live_check.setChecked(True)
        send_btn = QPushButton('Send command')
        send_btn.clicked.connect(self._send_now)
        grid.addWidget(self.live_check, row, 0, 1, 3)
        grid.addWidget(send_btn, row, 3, 1, 2)
        grid.setColumnStretch(1, 3)
        grid.setColumnStretch(3, 2)
        grid.setRowStretch(row + 1, 1)  # keep the rows together at the top
        return box

    def _build_preset_box(self):
        box = QGroupBox('Grasp presets')
        grid = QGridLayout(box)
        for i, name in enumerate(PRESETS):
            btn = QPushButton(name)
            btn.clicked.connect(lambda _, n=name: self.apply_preset(n))
            grid.addWidget(btn, i // 2, i % 2)
        return box

    def _build_sequence_box(self):
        box = QGroupBox('Motion sequence')
        lay = QGridLayout(box)
        self.play_btn = QPushButton('Play demo sequence')
        self.play_btn.setCheckable(True)
        self.play_btn.toggled.connect(self._toggle_sequence)
        self.step_spin = QDoubleSpinBox()
        self.step_spin.setRange(0.3, 10.0)
        self.step_spin.setValue(1.5)
        self.step_spin.setSuffix(' s / step')
        self.seq_label = QLabel(' -> '.join(DEMO_SEQUENCE))
        self.seq_label.setWordWrap(True)
        lay.addWidget(self.play_btn, 0, 0)
        lay.addWidget(self.step_spin, 0, 1)
        lay.addWidget(self.seq_label, 1, 0, 1, 2)
        return box

    def _build_hardware_box(self):
        box = QGroupBox('Hardware')
        lay = QGridLayout(box)
        enable = QPushButton('Enable servos')
        disable = QPushButton('Disable servos (limp)')
        home = QPushButton('Open all fingers')
        enable.clicked.connect(lambda: self._call(self.node.call_enable(True)))
        disable.clicked.connect(lambda: self._call(self.node.call_enable(False)))
        home.clicked.connect(self._home)
        self.status_label = QLabel('waiting for robothand_interface ...')
        self.status_label.setWordWrap(True)
        lay.addWidget(enable, 0, 0)
        lay.addWidget(disable, 0, 1)
        lay.addWidget(home, 1, 0, 1, 2)
        lay.addWidget(self.status_label, 2, 0, 1, 2)
        return box

    # ------------------------------------------------------------- slots
    def _slider_changed(self, value, spin, scale=10.0):
        spin.blockSignals(True)
        spin.setValue(value / scale)
        spin.blockSignals(False)
        self._mark_dirty()

    def _spin_changed(self, value, slider, scale=10.0):
        slider.blockSignals(True)
        slider.setValue(int(round(value * scale)))
        slider.blockSignals(False)
        self._mark_dirty()

    def _mark_dirty(self):
        if self.live_check.isChecked():
            self.dirty = True

    def _send_if_dirty(self):
        if self.dirty:
            self._send_now()

    def _send_now(self):
        self.dirty = False
        self.node.send_closures(self.closures())
        self.node.send_joints([THUMB_ROTATION_JOINT], [math.radians(self.thumb_spin.value())])

    def _call(self, ok):
        if not ok:
            self.status_label.setText('<font color="red">robothand_interface service not '
                                      'available</font>')

    def _home(self):
        self._set_closures([0.0] * len(FINGERS))
        self._call(self.node.call_trigger(self.node.home_cli))

    # ------------------------------------------------------------- values
    def closures(self):
        return [s.value() / 100.0 for s in self.spins]

    def _set_closures(self, closures):
        for slider, spin, c in zip(self.sliders, self.spins, closures):
            for w in (slider, spin):
                w.blockSignals(True)
            spin.setValue(100.0 * c)
            slider.setValue(int(round(1000 * c)))
            for w in (slider, spin):
                w.blockSignals(False)

    def _set_thumb_rotation(self, deg):
        for w in (self.thumb_slider, self.thumb_spin):
            w.blockSignals(True)
        self.thumb_spin.setValue(deg)
        self.thumb_slider.setValue(int(round(deg * 10)))
        for w in (self.thumb_slider, self.thumb_spin):
            w.blockSignals(False)

    def apply_preset(self, name):
        closures, rotation = PRESETS[name]
        self._set_closures(closures)
        self._set_thumb_rotation(rotation)
        self._send_now()

    # ------------------------------------------------------------- sequence
    def _toggle_sequence(self, running):
        if running:
            self.play_btn.setText('Stop sequence')
            start = (self.closures(), self.thumb_spin.value())
            self.sequence = {'from': start, 'index': 0, 't': 0.0}
            self.seq_timer.start(40)
        else:
            self.play_btn.setText('Play demo sequence')
            self.seq_timer.stop()
            self.sequence = None
            self.seq_label.setText(' -> '.join(DEMO_SEQUENCE))

    def _sequence_step(self):
        seq = self.sequence
        if seq is None:
            return
        duration = self.step_spin.value()
        seq['t'] += 0.04
        target_name = DEMO_SEQUENCE[seq['index']]
        target = PRESETS[target_name]
        s = min(seq['t'] / duration, 1.0)
        w = 0.5 - 0.5 * math.cos(math.pi * s)  # smooth start / stop
        c0, r0 = seq['from']
        closures = [a + w * (b - a) for a, b in zip(c0, target[0])]
        rotation = r0 + w * (target[1] - r0)
        self._set_closures(closures)
        self._set_thumb_rotation(rotation)
        self._send_now()
        self.seq_label.setText(f'step {seq["index"] + 1}/{len(DEMO_SEQUENCE)}: <b>{target_name}</b>')
        if s >= 1.0:
            seq['from'] = target
            seq['t'] = 0.0
            seq['index'] = (seq['index'] + 1) % len(DEMO_SEQUENCE)

    # ------------------------------------------------------------- feedback
    def update_view(self):
        state, age = self.node.latest_state()
        joints = self.node.snapshot_joints()
        for i, finger in enumerate(FINGERS):
            q = joints.get(finger + '_mcp_joint')
            self.angle_labels[i].setText('-' if q is None else f'{math.degrees(q):6.1f} deg')
        q = joints.get(THUMB_ROTATION_JOINT)
        self.thumb_actual.setText('-' if q is None else f'{math.degrees(q):6.1f} deg')

        if state is None or age > 1.0:
            self.status_label.setText('<font color="#c07000">robothand_interface not running '
                                      '(no robothand/state)</font>')
            for bar in self.bars:
                bar.setValue(0)
            return
        for bar, c in zip(self.bars, state.closure):
            bar.setValue(int(round(1000 * c)))
        if not self.synced and state.connected and len(state.closure) == len(FINGERS):
            # start from the current pose, so moving one slider does not move
            # the other fingers
            self._set_closures(state.closure)
            if q is not None:
                self._set_thumb_rotation(math.degrees(q))
            self.synced = True
        conn = '<font color="green">connected</font>' if state.connected \
            else '<font color="red">not connected</font>'
        power = 'enabled' if state.servos_enabled else 'disabled (limp)'
        fw = state.firmware or '-'
        self.status_label.setText(f'Board: {conn}<br>Firmware: {fw}<br>Servos: {power}<br>'
                                  'Servo angles: ' + ', '.join(f'{a:.0f}'
                                                               for a in state.servo_angles_deg))
