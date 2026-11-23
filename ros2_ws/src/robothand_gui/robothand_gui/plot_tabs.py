"""Live plots: joint angles and strain gauge measurements."""

import math

import numpy as np
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from python_qt_binding.QtWidgets import (
    QHBoxLayout, QHeaderView, QLabel, QPushButton, QTableWidget, QTableWidgetItem, QVBoxLayout,
    QWidget)

from .ros_node import FINGERS

COLORS = ['#d62728', '#1f77b4', '#2ca02c', '#ff7f0e', '#9467bd', '#8c564b', '#e377c2']


def _make_canvas(nrows=1, height_ratios=None):
    fig = Figure(figsize=(7, 4), tight_layout=True)
    canvas = FigureCanvasQTAgg(fig)
    if nrows == 1:
        axes = [fig.add_subplot(1, 1, 1)]
    else:
        gs = fig.add_gridspec(nrows, 1, height_ratios=height_ratios)
        axes = [fig.add_subplot(gs[i, 0]) for i in range(nrows)]
    return fig, canvas, axes


class JointPlotTab(QWidget):
    """Plot of the five actuated joints over time + table of all joints."""

    def __init__(self, node, parent=None):
        super().__init__(parent)
        self.node = node
        layout = QHBoxLayout(self)

        self.fig, self.canvas, (self.ax,) = _make_canvas()
        self.lines = [self.ax.plot([], [], color=COLORS[i], label=f)[0]
                      for i, f in enumerate(FINGERS)]
        self.ax.set_xlabel('time [s]')
        self.ax.set_ylabel('MCP joint angle [deg]')
        self.ax.set_ylim(-5, 95)
        self.ax.grid(True, alpha=0.3)
        self.ax.legend(loc='upper left', fontsize='small')
        layout.addWidget(self.canvas, 3)

        self.table = QTableWidget(0, 2)
        self.table.setHorizontalHeaderLabels(['Joint', 'Angle [deg]'])
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        layout.addWidget(self.table, 1)

    def update_view(self):
        t, v = self.node.joint_series()
        if len(t):
            tr = t - self.node.now()
            for i, line in enumerate(self.lines):
                line.set_data(tr, np.degrees(v[:, i]))
            self.ax.set_xlim(min(tr[0], -1.0), 0.0)
        self.canvas.draw_idle()

        joints = self.node.snapshot_joints()
        names = sorted(joints)
        if self.table.rowCount() != len(names):
            self.table.setRowCount(len(names))
        for row, name in enumerate(names):
            self.table.setItem(row, 0, QTableWidgetItem(name))
            self.table.setItem(row, 1, QTableWidgetItem(f'{math.degrees(joints[name]):8.2f}'))


class GaugePlotTab(QWidget):
    """Strain gauge values over time and as bar chart."""

    def __init__(self, node, parent=None):
        super().__init__(parent)
        self.node = node
        layout = QVBoxLayout(self)

        top = QHBoxLayout()
        self.info = QLabel('no strain gauge data (robothand/strain_gauges)')
        tare = QPushButton('Tare (set zero)')
        tare.clicked.connect(self._tare)
        top.addWidget(self.info, 1)
        top.addWidget(tare)
        layout.addLayout(top)

        self.fig, self.canvas, (self.ax, self.ax_bar) = _make_canvas(2, [3, 2])
        self.ax.set_xlabel('time [s]')
        self.ax.grid(True, alpha=0.3)
        self.ax_bar.grid(True, axis='y', alpha=0.3)
        layout.addWidget(self.canvas)
        self.lines = []
        self.bars = None
        self.names = []

    def _tare(self):
        if not self.node.call_trigger(self.node.tare_cli):
            self.info.setText('<font color="red">service robothand/tare not available</font>')

    def _setup_artists(self, names, unit):
        self.ax.cla()
        self.ax_bar.cla()
        self.lines = [self.ax.plot([], [], color=COLORS[i % len(COLORS)], label=n)[0]
                      for i, n in enumerate(names)]
        self.ax.set_xlabel('time [s]')
        self.ax.set_ylabel(f'value [{unit}]')
        self.ax.grid(True, alpha=0.3)
        self.ax.legend(loc='upper left', fontsize='small')
        self.bars = self.ax_bar.bar(names, np.zeros(len(names)),
                                    color=[COLORS[i % len(COLORS)] for i in range(len(names))])
        self.ax_bar.set_ylabel(f'current [{unit}]')
        self.ax_bar.grid(True, axis='y', alpha=0.3)
        self.names = list(names)

    def update_view(self):
        names, unit, t, v = self.node.gauge_series()
        if not len(t) or not names:
            return
        if names != self.names:
            self._setup_artists(names, unit)
        tr = t - self.node.now()
        n = min(len(self.lines), v.shape[1])
        for i in range(n):
            self.lines[i].set_data(tr, v[:, i])
        lo, hi = float(np.min(v[:, :n])), float(np.max(v[:, :n]))
        pad = max(1.0, 0.1 * (hi - lo))
        self.ax.set_xlim(min(tr[0], -1.0), 0.0)
        self.ax.set_ylim(lo - pad, hi + pad)
        for bar, value in zip(self.bars, v[-1, :n]):
            bar.set_height(value)
        self.ax_bar.set_ylim(min(0.0, lo - pad), max(0.0, hi + pad))
        self.info.setText(f'{len(names)} gauges, unit: {unit}, latest: ' +
                          ', '.join(f'{nm}={val:.1f}' for nm, val in zip(names, v[-1, :n])))
        self.canvas.draw_idle()
