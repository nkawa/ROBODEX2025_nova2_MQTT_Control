import multiprocessing.shared_memory as sm
import sys
import time
import threading
from collections import deque

import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

from config import (
    ABS_JOINT_LIMIT,
    SHM_NAME,
    SHM_SIZE,
    T_INTV,
)


class JointMonitorPlot(QtWidgets.QWidget):
    def __init__(
        self,
        max_points=1000,
    ):
        """関節角度の時系列データのプロット。"""
        super().__init__()
        self.n_joints = len(ABS_JOINT_LIMIT)
        self.max_points = max_points
        self.t_intv = T_INTV
        self.t_start = time.time()
        self.xdata = deque(maxlen=max_points)
        self.ydata = {
            'state': [deque(maxlen=max_points) for _ in range(self.n_joints)],
            'target': [deque(maxlen=max_points) for _ in range(self.n_joints)],
            'control': [deque(maxlen=max_points) for _ in range(self.n_joints)],
        }
        self.shm = sm.SharedMemory(SHM_NAME)
        self.pose = np.ndarray(
            (SHM_SIZE,), dtype=np.float32, buffer=self.shm.buf)
        self.data_lock = threading.Lock()
        self._stop_event = threading.Event()
        self.data_thread = threading.Thread(
            target=self.data_acquisition_loop, daemon=True)
        self.data_thread.start()
        self.plots = []
        self.curves = []
        layout = QtWidgets.QVBoxLayout()
        for i in range(self.n_joints):
            plot = pg.PlotWidget(title=f"Joint {i+1}")
            if i == 0:
                plot.addLegend()
            width = 3  # pixel
            curve_target = plot.plot(
                pen=pg.mkPen("r", width=width), name="target")
            curve_control = plot.plot(
                pen=pg.mkPen("g", width=width), name="control")
            curve_state = plot.plot(
                pen=pg.mkPen("b", width=width), name="state")
            self.plots.append(plot)
            self.curves.append({
                "state": curve_state,
                "target": curve_target,
                "control": curve_control
            })
            layout.addWidget(plot)
        self.setLayout(layout)

        # 前のupdate_plotが終わるまで待つのでその処理が長いと
        # 必ずしもt_intvの周期で呼ばれるわけではない
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(int(self.t_intv * 1000))

    def data_acquisition_loop(self):
        # 共有メモリから最近数件のデータを取得、高速なのでロックしてもOK
        while not self._stop_event.is_set():
            t = time.time() - self.t_start
            pose = self.pose.copy()
            with self.data_lock:
                self.xdata.append(t)
                for i in range(self.n_joints):
                    self.ydata['state'][i].append(float(pose[i]))
                    self.ydata['target'][i].append(float(pose[i+6]))
                    self.ydata['control'][i].append(float(pose[i+24]))
            # ループ時間が一定になるようにする
            t_after = time.time() - self.t_start
            t_elapsed = t_after - t
            t_wait = self.t_intv - t_elapsed
            if t_wait > 0:
                time.sleep(t_wait)

    def update_plot(self):
        # 最近数件のデータを取得、高速なのでロックしてもOK
        with self.data_lock:
            x = np.array(self.xdata)
            y = {}
            for i in range(self.n_joints):
                for k in self.ydata:
                    y_ = np.array(self.ydata[k][i])
                    if k not in y:
                        y[k] = [y_]
                    else:
                        y[k].append(y_)
        # 描画、低速 (30 ms周期くらい)なのでロックしないほうがいい
        for i in range(self.n_joints):
            for k in self.ydata:
                self.curves[i][k].setData(x, y[k][i])
        if self.pose[32] == 1:
            self.shm.close()
            time.sleep(1)


def run_joint_monitor_gui():
    app = QtWidgets.QApplication(sys.argv)
    win = JointMonitorPlot()
    win.setWindowTitle('Cobotta Pro Joint Monitor')
    win.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    run_joint_monitor_gui()
