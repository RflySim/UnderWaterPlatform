import time
import sys
import numpy as np
import cv2
import matplotlib.pyplot as plt

# rflysim 相关模块（保持原样）
import UE4CtrlAPI
import PX4MavCtrlV4 as PX4MavCtrl
import VisionCaptureApi


# 预计算角度表
def precompute_angle_table(H, W, fov_deg):
    fov_rad = np.deg2rad(fov_deg)
    cx, cy = W / 2, H / 2
    fx = W / (2 * np.tan(fov_rad / 2))

    u = np.arange(W)
    v = np.arange(H)
    uu, vv = np.meshgrid(u, v)

    # 预计算每个像素的水平角度 θ
    theta_table = np.arctan2((uu - cx) / fx, np.ones_like(uu))
    return theta_table.astype(np.float32)


# 优化版深度图 → 声纳图
def depth_to_sonar_fast(depth_map, theta_table, range_max, azimuth_bins, range_bins, attenuation_alpha):
    valid = (depth_map > 0) & (depth_map < range_max)
    if not np.any(valid):
        return np.zeros((range_bins, azimuth_bins), dtype=np.float32)

    depth_valid = depth_map[valid]
    theta_valid = theta_table[valid]

    theta_min = theta_table.min()
    theta_max = theta_table.max()

    # bin 映射
    azimuth_idx = ((theta_valid - theta_min) / (theta_max - theta_min) * azimuth_bins).astype(np.int32)
    range_idx = (depth_valid / range_max * range_bins).astype(np.int32)
    np.clip(azimuth_idx, 0, azimuth_bins - 1, out=azimuth_idx)
    np.clip(range_idx, 0, range_bins - 1, out=range_idx)

    # 衰减
    intensity = np.exp(-attenuation_alpha * depth_valid)

    # np.bincount 快速累加
    flat_idx = range_idx * azimuth_bins + azimuth_idx
    sonar = np.bincount(flat_idx, weights=intensity, minlength=range_bins * azimuth_bins)
    sonar = sonar.reshape(range_bins, azimuth_bins)
    return sonar


# 初始化绘图（只建一次）
def init_sonar_plot(fov_deg, range_max, azimuth_bins, range_bins):
    fig = plt.figure(figsize=(8, 6))
    ax = plt.subplot(111, polar=True)

    theta_min = -fov_deg / 2
    theta_max = fov_deg / 2
    theta_grid = np.linspace(np.deg2rad(theta_min), np.deg2rad(theta_max), azimuth_bins)
    range_grid = np.linspace(0, range_max, range_bins)

    R, Theta = np.meshgrid(range_grid, theta_grid)

    # 初始化全 0 图
    mesh = ax.pcolormesh(Theta, R, np.zeros((azimuth_bins, range_bins)), shading='auto', cmap='gray')

    ax.set_thetamin(theta_min)
    ax.set_thetamax(theta_max)
    ax.set_theta_zero_location('N')
    ax.set_theta_direction(-1)
    ax.set_rlabel_position(-22.5)

    plt.colorbar(mesh, ax=ax, label='Log-Normalized Intensity')
    plt.title('Imaging Sonar (Fan-shaped)')
    plt.tight_layout()
    plt.pause(0.01)
    return fig, ax, mesh


# 更新绘图（高效复用 mesh）
def update_sonar_plot(mesh, sonar):
    eps = 1e-3
    sonar_log = np.log(sonar + eps)
    sonar_norm = (sonar_log - sonar_log.min()) / (sonar_log.max() - sonar_log.min() + eps)

    mesh.set_array(sonar_norm.T.ravel())
    mesh.autoscale()  # 自动更新颜色范围
    mesh.figure.canvas.draw_idle()
    plt.pause(0.001)


# 主循环
if __name__ == "__main__":
    # 初始化 Rflysim
    mav = PX4MavCtrl.PX4MavCtrler(20100)
    vis = VisionCaptureApi.VisionCaptureApi()
    ue = UE4CtrlAPI.UE4CtrlAPI()
    time.sleep(2)

    vis.jsonLoad()
    time.sleep(2)

    isSuss = vis.sendReqToUE4()
    if not isSuss:
        print('无图')
        sys.exit(0)
    vis.startImgCap()
    vis.sendImuReqCopterSim()
    time.sleep(2)

    # 获取一帧，确定分辨率
    depth_raw = vis.Img[0]
    H, W = depth_raw.shape[:2]

    fov_deg = 90.0
    range_max = 40.0
    azimuth_bins = 300
    range_bins = 400
    attenuation_alpha = 0.2

    # 预计算角度表（一次性）
    theta_table = precompute_angle_table(H, W, fov_deg)

    # 初始化 sonar 绘图
    fig, ax, mesh = init_sonar_plot(fov_deg, range_max, azimuth_bins, range_bins)

    last_time = time.time()
    print("开始实时显示声纳图... 按 ESC 退出")

    while True:
        depth_raw = vis.Img[0]  # mm
        depth_m = depth_raw.astype(np.float32) / 1000.0

        sonar = depth_to_sonar_fast(
            depth_map=depth_m,
            theta_table=theta_table,
            range_max=range_max,
            azimuth_bins=azimuth_bins,
            range_bins=range_bins,
            attenuation_alpha=attenuation_alpha
        )

        update_sonar_plot(mesh, sonar)

        fps = 1 / (time.time() - last_time)
        last_time = time.time()
        print(f"FPS: {fps:.1f}\r", end="")

        key = cv2.waitKey(1)
        if key == 27:  # ESC
            break

    cv2.destroyAllWindows()
    plt.close('all')
