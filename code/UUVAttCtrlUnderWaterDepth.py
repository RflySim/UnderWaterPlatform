import time
import sys
import os
import numpy as np
import cv2
import datetime
from threading import Thread
import matplotlib.pyplot as plt
import PX4MavCtrlV4 as PX4MavCtrl
from PyQt5.QtWidgets import QApplication, QWidget, QHBoxLayout, QPushButton, QVBoxLayout, QLabel
from PyQt5.QtCore import QTimer, Qt
import pyqtgraph as pg



class DepthSensorSimulator:
    def __init__(self, fs=10.0, b0=0.1, sigma_b=0.02, sigma_s=0.001,
                 sigma0=0.01, alpha=0.0005, A_w=0.3, f_w=0.2, H_wave=15.0, seed=42):
        self.fs = fs
        self.Ts = 1.0 / fs
        self.b0 = b0
        self.sigma_b = sigma_b
        self.sigma_s = sigma_s
        self.sigma0 = sigma0
        self.alpha = alpha
        self.A_w = A_w
        self.f_w = f_w
        self.H_wave = H_wave

        np.random.seed(seed)
        self.b = np.random.normal(self.b0, self.sigma_b)
        self.k_s = np.random.normal(0, self.sigma_s)

        self.last_sample_time = -1.0
        self.last_output = 0.0

    def update(self, z_true, t):
        self.b = np.random.normal(self.b0, self.sigma_b)
        self.k_s = np.random.normal(0, self.sigma_s)

        if z_true < self.H_wave:
            phi_w = np.random.uniform(0, 2 * np.pi)
            d_wave = z_true + self.A_w * np.sin(2 * np.pi * self.f_w * t + phi_w)
        else:
            d_wave = z_true

        d_bias_scale = (d_wave + self.b) * (1 + self.k_s)
        sigma_n = self.sigma0 + self.alpha * z_true
        epsilon = np.random.normal(0, sigma_n)
        d_continuous = d_bias_scale + epsilon

        if t >= self.last_sample_time + self.Ts:
            self.last_output = d_continuous
            self.last_sample_time = t

        return d_continuous, self.last_output


class DepthSensorGUI(QWidget):
    def __init__(self, mav):
        super().__init__()
        self.setWindowTitle("Depth Sensor Simulator (QGC-style)")
        self.resize(1000, 600)

        self.mav = mav

        # 仿真参数
        self.T_WINDOW = 10.0   # 显示最近 20 秒
        self.DT = 0.05         # 仿真步长 (s)

        # 初始化传感器（使用你已有的类）
        self.sensor = DepthSensorSimulator(fs=10.0, b0=0.01, sigma_b=0.02, sigma_s=0.001, sigma0=0.01, alpha=0.0005, A_w=0.3, f_w=0.6, H_wave=10.0, seed=123)

        # 数据缓冲区（固定长度，用 numpy array 提升性能）
        self.max_points = int(self.T_WINDOW / self.DT) + 10
        self.times = np.zeros(self.max_points)
        self.true_vals = np.zeros(self.max_points)
        self.cont_vals = np.zeros(self.max_points)
        self.out_vals = np.zeros(self.max_points)
        self.ptr = 0  # 当前写入位置

        self._init_ui()
        self._init_plot()

        # 启动定时器（每 DT 秒更新一次）
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_simulation)
        self.timer.start(int(self.DT * 1000))  # ms

    def _init_ui(self):
        main_layout = QVBoxLayout()

        # --- 右侧绘图区域 ---
        self.plot_widget = pg.PlotWidget(title="Depth Sensor Data")
        self.plot_widget.setLabel('left', "Depth", units='m')
        self.plot_widget.setLabel('bottom', "Time", units='s')
        self.plot_widget.setYRange(0, 100)
        self.plot_widget.setXRange(0, self.T_WINDOW)
        self.plot_widget.showGrid(x=True, y=True, alpha=0.5)
        main_layout.addWidget(self.plot_widget)
        # --- 底部按钮区域 ---
        save_button_layout = QHBoxLayout()
        save_button_layout.addStretch()
        self.button1 = QPushButton('Save Data', self)
        self.button1.clicked.connect(self.save_data)
        save_button_layout.addWidget(self.button1)
        save_button_layout.addStretch()
        main_layout.addLayout(save_button_layout)

        self.setLayout(main_layout)

    def _init_plot(self):
        pen_true = pg.mkPen(color='g', width=2)
        pen_cont = pg.mkPen(color='b', width=1.5)
        pen_out = pg.mkPen(color='r', width=2, style=Qt.DashLine)

        self.curve_true = self.plot_widget.plot(pen=pen_true, name='True Depth')
        self.curve_cont = self.plot_widget.plot(pen=pen_cont, name='Continuous')
        self.curve_out = self.plot_widget.plot(pen=pen_out, name='Sampled Output')

    def update_simulation(self):
        z_true = self.mav.truePosNED[2]
        t = self.mav.uavTimeStmp 
        d_cont, d_out = self.sensor.update(z_true, t)

        # 写入环形缓冲区
        idx = self.ptr % self.max_points
        self.times[idx] = t
        self.true_vals[idx] = z_true
        self.cont_vals[idx] = d_cont
        self.out_vals[idx] = d_out
        self.ptr += 1

        # 计算显示范围（最近 T_WINDOW 秒）
        start_idx = max(0, self.ptr - self.max_points)
        end_idx = self.ptr
        if end_idx - start_idx > len(self.times):
            start_idx = end_idx - len(self.times)

        # 提取有效数据
        valid_n = min(self.ptr, self.max_points)
        if self.ptr <= self.max_points:
            self.t_data = self.times[:valid_n] - t + self.T_WINDOW
            self.true_data = self.true_vals[:valid_n]
            self.cont_data = self.cont_vals[:valid_n]
            self.out_data = self.out_vals[:valid_n]
        else:
            # 环形缓冲：拼接后半段 + 前半段
            split = self.ptr % self.max_points
            self.t_data = np.concatenate([self.times[split:], self.times[:split]]) - t + self.T_WINDOW
            self.true_data = np.concatenate([self.true_vals[split:], self.true_vals[:split]])
            self.cont_data = np.concatenate([self.cont_vals[split:], self.cont_vals[:split]])
            self.out_data = np.concatenate([self.out_vals[split:], self.out_vals[:split]])

        # 更新曲线
        self.curve_true.setData(self.t_data, self.true_data)
        self.curve_cont.setData(self.t_data, self.cont_data)
        self.curve_out.setData(self.t_data, self.out_data)
    
    def save_data(self):
        # 保存数组为csv文件
        folder = r"E:\研究生学习\水下机器人\uuv20240309\under_water_depth_data"
        if not os.path.exists(folder):
            os.makedirs(folder)

        folder = os.path.join(folder, datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S'))
        if not os.path.exists(folder):
            os.makedirs(folder)

        np.savetxt(os.path.join(folder, 'time.csv'), self.t_data, delimiter=',')
        np.savetxt(os.path.join(folder, 'true_data.csv'), self.true_data, delimiter=',')
        np.savetxt(os.path.join(folder, 'cont_data.csv'), self.cont_data, delimiter=',')
        np.savetxt(os.path.join(folder, 'out_data.csv'), self.out_data, delimiter=',')

        print("Saved success:")
        print(os.path.join(folder, 'time.csv'))
        print(os.path.join(folder, 'true_data.csv'))
        print(os.path.join(folder, 'cont_data.csv'))
        print(os.path.join(folder, 'out_data.csv'))


if __name__ == "__main__":
    # Create MAVLink control API instance
    mav1 = PX4MavCtrl.PX4MavCtrler(20100)

    # Init MAVLink data receiving loop
    mav1.InitMavLoop()

    app = QApplication(sys.argv)
    window = DepthSensorGUI(mav1)
    window.show()
    sys.exit(app.exec_())

