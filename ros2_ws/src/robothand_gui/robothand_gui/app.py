"""Robot hand GUI - entry point.

    ros2 run robothand_gui robothand_gui [--ros-args -p dh_file:=/path/dh.yaml]

Tabs:
  Control         finger sliders, grasp presets, demo sequence, servo power
  Joint angles    live plot of the joint angles (from /joint_states)
  Strain gauges   live plot of the strain gauge values, tare
  Kinematics (DH) Denavit-Hartenberg table, A_i matrices, comparison with TF
"""

import os
import signal
import sys
import threading

import rclpy
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import QApplication, QLabel, QMainWindow, QTabWidget
from rclpy.executors import SingleThreadedExecutor
from rclpy.utilities import remove_ros_args

from .control_tab import ControlTab
from .dh_tab import DHTab
from .kinematics import HandKinematics
from .plot_tabs import GaugePlotTab, JointPlotTab
from .ros_node import GuiNode


def default_dh_file() -> str:
    try:
        share = get_package_share_directory('robothand_description')
    except PackageNotFoundError:
        return ''
    return os.path.join(share, 'config', 'dh_parameters.yaml')


class MainWindow(QMainWindow):

    def __init__(self, node: GuiNode, kinematics: HandKinematics):
        super().__init__()
        self.node = node
        self.setWindowTitle('Robot Hand Control')
        self.resize(1250, 780)

        self.tabs = QTabWidget()
        self.control = ControlTab(node)
        self.joints = JointPlotTab(node)
        self.gauges = GaugePlotTab(node)
        self.tabs.addTab(self.control, 'Control')
        self.tabs.addTab(self.joints, 'Joint angles')
        self.tabs.addTab(self.gauges, 'Strain gauges')
        if kinematics is not None:
            self.dh = DHTab(node, kinematics)
            self.tabs.addTab(self.dh, 'Kinematics (DH)')
        else:
            self.dh = None
            self.tabs.addTab(QLabel('DH parameter file not found'), 'Kinematics (DH)')
        self.setCentralWidget(self.tabs)
        self.status = QLabel()
        self.statusBar().addWidget(self.status)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._update)
        self.timer.start(100)  # 10 Hz GUI refresh

    def _update(self):
        current = self.tabs.currentWidget()
        self.control.update_view()  # status is always kept up to date
        if current is not self.control and hasattr(current, 'update_view'):
            current.update_view()
        state, age = self.node.latest_state()
        if state is None or age > 1.0:
            self.status.setText('robothand_interface: not running')
        else:
            self.status.setText('robothand_interface: ' + (
                f'connected to {state.firmware}' if state.connected else 'board not connected'))


def main(args=None):
    rclpy.init(args=args)
    node = GuiNode()
    node.declare_parameter('dh_file', default_dh_file())
    dh_file = node.get_parameter('dh_file').value
    kinematics = None
    if dh_file and os.path.isfile(dh_file):
        kinematics = HandKinematics.from_yaml(dh_file)
        node.get_logger().info(f'DH parameters: {dh_file}')
    else:
        node.get_logger().error(f'DH parameter file not found: "{dh_file}"')

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    app = QApplication(remove_ros_args(sys.argv))
    window = MainWindow(node, kinematics)
    window.show()

    # let Ctrl+C close the window (Qt blocks Python signal handlers otherwise)
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    keep_alive = QTimer()
    keep_alive.timeout.connect(lambda: None)
    keep_alive.start(200)

    exit_code = app.exec_()
    executor.shutdown()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
