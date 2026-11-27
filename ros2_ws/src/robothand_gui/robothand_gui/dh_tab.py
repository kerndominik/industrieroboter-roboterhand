"""Kinematics tab: Denavit-Hartenberg table, transforms and 3D view.

The finger tip pose is computed with the DH parameters (forward kinematics)
and compared with the transform that robot_state_publisher computes from the
URDF (TF). Both must agree - a nice check of the DH table.
"""

import math

import numpy as np
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtWidgets import (
    QCheckBox, QComboBox, QHBoxLayout, QHeaderView, QLabel, QPlainTextEdit, QTableWidget,
    QTableWidgetItem, QVBoxLayout, QWidget)

from .kinematics import format_matrix
from .plot_tabs import COLORS

PALM_OUTLINE_MM = np.array([[-115, 0, 15], [0, 0, 15], [0, 157, 15], [-115, 157, 15],
                            [-115, 0, 15]], dtype=float)
AXIS_LEN = 0.012  # length of the drawn frame axes [m]


class DHTab(QWidget):

    def __init__(self, node, kinematics, parent=None):
        super().__init__(parent)
        self.node = node
        self.kin = kinematics
        names = list(self.kin.fingers)

        layout = QHBoxLayout(self)
        left = QVBoxLayout()
        row = QHBoxLayout()
        row.addWidget(QLabel('Finger:'))
        self.combo = QComboBox()
        self.combo.addItems(names)
        self.combo.setCurrentText('index' if 'index' in names else names[0])
        row.addWidget(self.combo, 1)
        left.addLayout(row)

        left.addWidget(QLabel('<b>DH parameters</b> (standard convention: '
                              'A<sub>i</sub> = Rot<sub>z</sub>(&theta;<sub>i</sub>) '
                              'Trans<sub>z</sub>(d<sub>i</sub>) Trans<sub>x</sub>(a<sub>i</sub>) '
                              'Rot<sub>x</sub>(&alpha;<sub>i</sub>), '
                              '&theta;<sub>i</sub> = q<sub>i</sub> + offset)'))
        self.table = QTableWidget(0, 6)
        self.table.setHorizontalHeaderLabels(
            ['Joint', 'q [deg]', 'θ = q+off [deg]', 'd [mm]', 'a [mm]', 'α [deg]'])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.verticalHeader().setVisible(True)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.setMaximumHeight(140)
        left.addWidget(self.table)

        self.text = QPlainTextEdit()
        self.text.setReadOnly(True)
        font = QFont('Monospace')
        font.setStyleHint(QFont.TypeWriter)
        font.setPointSize(9)
        self.text.setFont(font)
        left.addWidget(self.text, 1)

        self.compare = QLabel()
        self.compare.setWordWrap(True)
        left.addWidget(self.compare)
        layout.addLayout(left, 2)

        right = QVBoxLayout()
        self.frames_check = QCheckBox('Show DH frames of the selected finger (x red, y green, z blue)')
        self.frames_check.setChecked(True)
        right.addWidget(self.frames_check)
        self.fig = Figure(figsize=(6, 6), tight_layout=True)
        self.canvas = FigureCanvasQTAgg(self.fig)
        self.ax = self.fig.add_subplot(1, 1, 1, projection='3d')
        right.addWidget(self.canvas, 1)
        layout.addLayout(right, 3)
        self._setup_3d()

    # ------------------------------------------------------------------ 3D
    def _setup_3d(self):
        ax = self.ax
        ax.plot(PALM_OUTLINE_MM[:, 0], PALM_OUTLINE_MM[:, 1], PALM_OUTLINE_MM[:, 2],
                color='gray', lw=1.0)
        self.chain_lines = {}
        for i, name in enumerate(self.kin.fingers):
            self.chain_lines[name], = ax.plot([], [], [], '-o', color=COLORS[i % len(COLORS)],
                                              lw=2.5, ms=4, label=name)
        self.tf_tips, = ax.plot([], [], [], 'kx', ms=8, mew=2, label='tip from TF (URDF)')
        self.axis_lines = [ax.plot([], [], [], color=c, lw=1.5)[0]
                           for _ in range(8) for c in ('r', 'g', 'b')]
        ax.set_xlabel('x [mm]')
        ax.set_ylabel('y [mm]')
        ax.set_zlabel('z [mm]')
        ax.set_xlim(-150, 70)
        ax.set_ylim(-20, 280)
        ax.set_zlim(-70, 150)
        try:
            ax.set_box_aspect((220, 300, 220))
        except AttributeError:  # old matplotlib
            pass
        ax.view_init(elev=30, azim=-130)
        ax.legend(loc='upper left', fontsize='x-small')

    def update_view(self):
        joints = self.node.snapshot_joints()
        selected = self.combo.currentText()
        tf_points = []
        for name, chain in self.kin.fingers.items():
            q = chain.q_from_joint_states(joints)
            frames = chain.frames(q)
            pts = np.array([f[:3, 3] for f in frames]) * 1000.0
            self.chain_lines[name].set_data_3d(pts[:, 0], pts[:, 1], pts[:, 2])
            p_tf = self.node.lookup(self.kin.base_frame, chain.tip_frame)
            if p_tf is not None:
                tf_points.append(p_tf * 1000.0)
            if name == selected:
                self._update_selected(chain, q, frames, p_tf)
        if tf_points:
            tp = np.array(tf_points)
            self.tf_tips.set_data_3d(tp[:, 0], tp[:, 1], tp[:, 2])
        self.canvas.draw_idle()

    def _update_selected(self, chain, q, frames, p_tf):
        # DH table
        thetas = chain.thetas(q)
        self.table.setRowCount(len(chain.rows))
        for i, (row, qi, th) in enumerate(zip(chain.rows, q, thetas)):
            values = [chain.joints[i], f'{math.degrees(qi):.2f}', f'{math.degrees(th):.2f}',
                      f'{row.d * 1000:.2f}', f'{row.a * 1000:.2f}',
                      f'{math.degrees(row.alpha):.1f}']
            for c, val in enumerate(values):
                self.table.setItem(i, c, QTableWidgetItem(val))

        # matrices
        lines = [f'T_palm_0 (constant base transform, translation in mm):',
                 format_matrix(chain.base_transform), '']
        for i, a in enumerate(chain.link_transforms(q), start=1):
            lines += [f'A_{i}  ({chain.joints[i - 1]}):', format_matrix(a), '']
        tip = frames[-1]
        lines += [f'T_palm_tip = T_palm_0 * ' + ' * '.join(f'A_{i}'
                                                         for i in range(1, len(q) + 1)) + ':',
                  format_matrix(tip)]
        self.text.setPlainText('\n'.join(lines))

        # comparison with TF
        p_dh = tip[:3, 3] * 1000.0
        txt = (f'<b>Tip position (DH):</b> x={p_dh[0]:.2f} y={p_dh[1]:.2f} '
               f'z={p_dh[2]:.2f} mm<br>')
        if p_tf is None:
            txt += '<b>TF:</b> <font color="#c07000">no transform ' \
                   f'{self.kin.base_frame} -> {chain.tip_frame} (robot_state_publisher running?)</font>'
        else:
            p = p_tf * 1000.0
            err = float(np.linalg.norm(p - p_dh))
            color = 'green' if err < 0.5 else 'red'
            txt += (f'<b>Tip position (TF / URDF):</b> x={p[0]:.2f} y={p[1]:.2f} z={p[2]:.2f} mm'
                    f'<br><b>Difference:</b> <font color="{color}">{err:.3f} mm</font>')
        self.compare.setText(txt)

        # frame axes
        show = self.frames_check.isChecked()
        k = 0
        for f in frames if show else []:
            o = f[:3, 3]
            for c in range(3):
                e = o + f[:3, c] * AXIS_LEN
                seg = np.array([o, e]) * 1000.0
                if k < len(self.axis_lines):
                    self.axis_lines[k].set_data_3d(seg[:, 0], seg[:, 1], seg[:, 2])
                k += 1
        for line in self.axis_lines[k:]:
            line.set_data_3d([], [], [])
