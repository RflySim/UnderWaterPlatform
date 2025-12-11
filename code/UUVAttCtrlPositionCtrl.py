import math


def position_controller(p, p_d, k_T=0.3):
    """
    简单位置控制器（NED坐标系）
    输入:
        p    : 当前位置 [x, y, z]
        att  : 当前姿态欧拉角 [roll, pitch, yaw] (deg)
        p_d  : 目标位置 [x_d, y_d, z_d]
        k_T  : 油门比例系数
    输出:
        cmd_att : 目标姿态 [roll, pitch, yaw] (deg)
        T_c     : 油门 [0, 1]
    """
    # --- 位置误差 ---
    e_p = np.array(p_d) - np.array(p)
    dist = np.linalg.norm(e_p) + 1e-6

    # --- 期望方向 ---
    dir_unit = e_p / dist

    # --- 偏航角 (yaw) ---
    psi_c = np.degrees(np.arctan2(e_p[1], e_p[0]))

    # --- 俯仰角 (pitch) ---
    # NED坐标中 z 向下，因此如果目标在下方（z_d>z），应产生正俯仰
    theta_c = np.degrees(np.arctan2(-e_p[2], np.sqrt(e_p[0]**2 + e_p[1]**2)))

    # --- 滚转角 (roll) 固定为0 ---
    phi_c = 0.0

    # --- 油门 ---
    T_c = np.clip(k_T * dist, 0, 1)

    cmd_att = [phi_c, theta_c, psi_c]
    if psi_c >= 90 or psi_c <= - 90:
        theta_c = -theta_c
    return phi_c, theta_c, psi_c, T_c


# import required libraries
import time
import numpy as np

# import RflySim APIs
import PX4MavCtrlV4 as PX4MavCtrl

# Create MAVLink control API instance
mav1 = PX4MavCtrl.PX4MavCtrler(20100)

# Init MAVLink data receiving loop
mav1.InitMavLoop()

lastTime = time.time()
startTime = time.time()

# time interval of the timer
timeInterval = 1/30.0 #here is 0.0333s (30Hz)

flag1 = False  # 解锁标记
flag2 = False  # 任务开始时间记录标记
flag3 = False  # 文件保存记录
# 路径相关参数
i = 0  # 当前迭代次数
count = 5  # 路径重复次数
cowTime = 50  # 纵向的时长
rowTime = 10  # 横向的时长
T = 2 * (cowTime + rowTime)  # 路径周期
state = 1  # 当前的行进状态
missionStartTime = time.time()

uavTimeStmp = np.array([[0]])
trueTimeStmp = np.array([[0]])
uavAngEular = np.array([[0, 0, 0]])  # Estimated Eular angles from PX4
trueAngEular = np.array([[0, 0, 0]]) # True simulated Eular angles from CopterSim's DLL model
uavAngRate = np.array([[0, 0, 0]])  # Estimated angular rate from PX4
trueAngRate = np.array([[0, 0, 0]]) # True simulated angular rate from CopterSim's DLL model
uavPosNED = np.array([[0, 0, 0]]) # Estimated Local Pos (related to takeoff position) from PX4 in NED frame
truePosNED = np.array([[0, 0, 0]]) # True simulated position (related to UE4 map center) from CopterSim's DLL model
uavVelNED = np.array([[0, 0, 0]]) # Estimated local velocity from PX4 in NED frame
trueVelNED = np.array([[0, 0, 0]]) # True simulated speed from CopterSim's DLL model  in NED frame
uavAngQuatern = np.array([[0, 0, 0, 0]]) # Estimated AngQuatern from PX4
trueAngQuatern = np.array([[0, 0, 0, 0]]) # True simulated AngQuatern from CopterSim's DLL model
uavMotorRPMS = np.array([[0, 0, 0, 0, 0, 0, 0, 0]]) # Estimated MotorPWMs from PX4
trueMotorRPMS = np.array([[0, 0, 0, 0, 0, 0, 0, 0]]) # True simulated MotorRPMS from CopterSim's DLL model
uavAccB = np.array([[0, 0, 0]]) # Estimated acc from PX4
trueAccB = np.array([[0, 0, 0]]) # True simulated acc from CopterSim's DLL model

uavGyro = np.array([[0, 0, 0]]) # Estimated Gyro from PX4
uavMag = np.array([[0, 0, 0]]) # Estimated Gyro from PX4
uavVibr = np.array([[0, 0, 0]]) # Estimated vibration xyz from PX4
uavPosGPS = np.array([[0, 0, 0, 0, 0, 0, 0, 0, 0]]) # Estimated GPS position from PX4 in NED frame,lat lon alt relative_alt vx vy vz hdg
uavPosGPSHome = np.array([[0, 0, 0]]) # Estimated GPS home (takeoff) position from PX4 in NED frame
uavGlobalPos = np.array([[0, 0, 0]]) # Estimated global position from PX4 that transferred to UE4 map

roll = 0
pitch = 0
yaw = 0
thrust = 0.6
flag_roll = True
flag_pitch = True
flag_yaw = True
flag_thrust = True

x_d = 0
y_d = 0
z_d = 0
theta = 0
R = 5

while True:
    lastTime = lastTime + timeInterval
    sleepTime = lastTime - time.time()
    if sleepTime > 0:
        time.sleep(sleepTime) # sleep until the desired clock
    else:
        lastTime = time.time()

    # The following code will be executed 30Hz (0.0333s)
    # Create the mission to vehicle 1
    if flag1 == False and time.time() - startTime > 5:
        # The following code will be executed at 5s
        print("5s, Arm the drone!")
        print("开始进入Offboard模式")
        mav1.initOffboard()
        time.sleep(0.5)
        mav1.SendMavArm(True) # Arm the drone
        time.sleep(0.5)
        print("Armed the drone!")
        ArmedTime = time.time()
        flag1 = True

    # 如果没解锁，就等待解锁
    if flag1 == False:
        continue

    # 如果任务没开始，就开始任务，并记录任务开始时间
    if flag2 == False:
        missionStartTime = time.time()
        flag2 = True

    
    x_d = R * math.cos(theta)
    y_d = R * math.sin(theta)
    p_d = [x_d, y_d, z_d]
    theta += 0.1

    while (np.linalg.norm(np.array(p_d) - np.array(mav1.uavPosNED)) > 1):
        print(mav1.uavPosNED, math.sqrt(mav1.uavPosNED[0]**2 + mav1.uavPosNED[1]**2 + mav1.uavPosNED[2]**2))
        roll, pitch, yaw, thrust = position_controller(mav1.uavPosNED, p_d)
        mav1.SendAttPX4([roll, pitch, yaw], 0.6)

    #     uavTimeStmp = np.vstack((uavTimeStmp, np.array([[mav1.uavTimeStmp]])))
    #     trueTimeStmp = np.vstack((trueTimeStmp, np.array([[mav1.trueTimeStmp]])))

    #     uavAngEular = np.vstack((uavAngEular, np.array([mav1.uavAngEular])))
    #     trueAngEular = np.vstack((trueAngEular, np.array([mav1.trueAngEular])))

    #     uavAngRate = np.vstack((uavAngRate, np.array([mav1.uavAngRate])))
    #     trueAngRate = np.vstack((trueAngRate, np.array([mav1.trueAngRate])))

    #     uavPosNED = np.vstack((uavPosNED, np.array([mav1.uavPosNED])))
    #     truePosNED = np.vstack((truePosNED, np.array([mav1.truePosNED])))

    #     uavVelNED = np.vstack((uavVelNED, np.array([mav1.uavVelNED])))
    #     trueVelNED = np.vstack((trueVelNED, np.array([mav1.trueVelNED])))

    #     uavAngQuatern = np.vstack((uavAngQuatern, np.array([mav1.uavAngQuatern])))
    #     trueAngQuatern = np.vstack((trueAngQuatern, np.array([mav1.trueAngQuatern])))

    #     uavMotorRPMS = np.vstack((uavMotorRPMS, np.array([mav1.uavMotorRPMS])))
    #     trueMotorRPMS = np.vstack((trueMotorRPMS, np.array([mav1.trueMotorRPMS])))

    #     uavAccB = np.vstack((uavAccB, np.array([mav1.uavAccB])))
    #     trueAccB = np.vstack((trueAccB, np.array([mav1.trueAccB])))

    #     uavGyro = np.vstack((uavGyro, np.array([mav1.uavGyro])))
    #     uavMag = np.vstack((uavMag, np.array([mav1.uavMag])))
    #     uavVibr = np.vstack((uavVibr, np.array([mav1.uavVibr])))
    #     uavPosGPS = np.vstack((uavPosGPS, np.array([mav1.uavPosGPS])))
    #     uavPosGPSHome = np.vstack((uavPosGPSHome, np.array([mav1.uavPosGPSHome])))
    #     uavGlobalPos = np.vstack((uavGlobalPos, np.array([mav1.uavGlobalPos])))


    # uavTimeStmp = np.vstack((uavTimeStmp, np.array([[mav1.uavTimeStmp]])))
    # trueTimeStmp = np.vstack((trueTimeStmp, np.array([[mav1.trueTimeStmp]])))

    # uavAngEular = np.vstack((uavAngEular, np.array([mav1.uavAngEular])))
    # trueAngEular = np.vstack((trueAngEular, np.array([mav1.trueAngEular])))

    # uavAngRate = np.vstack((uavAngRate, np.array([mav1.uavAngRate])))
    # trueAngRate = np.vstack((trueAngRate, np.array([mav1.trueAngRate])))

    # uavPosNED = np.vstack((uavPosNED, np.array([mav1.uavPosNED])))
    # truePosNED = np.vstack((truePosNED, np.array([mav1.truePosNED])))

    # uavVelNED = np.vstack((uavVelNED, np.array([mav1.uavVelNED])))
    # trueVelNED = np.vstack((trueVelNED, np.array([mav1.trueVelNED])))

    # uavAngQuatern = np.vstack((uavAngQuatern, np.array([mav1.uavAngQuatern])))
    # trueAngQuatern = np.vstack((trueAngQuatern, np.array([mav1.trueAngQuatern])))

    # uavMotorRPMS = np.vstack((uavMotorRPMS, np.array([mav1.uavMotorRPMS])))
    # trueMotorRPMS = np.vstack((trueMotorRPMS, np.array([mav1.trueMotorRPMS])))

    # uavAccB = np.vstack((uavAccB, np.array([mav1.uavAccB])))
    # trueAccB = np.vstack((trueAccB, np.array([mav1.trueAccB])))

    # uavGyro = np.vstack((uavGyro, np.array([mav1.uavGyro])))
    # uavMag = np.vstack((uavMag, np.array([mav1.uavMag])))
    # uavVibr = np.vstack((uavVibr, np.array([mav1.uavVibr])))
    # uavPosGPS = np.vstack((uavPosGPS, np.array([mav1.uavPosGPS])))
    # uavPosGPSHome = np.vstack((uavPosGPSHome, np.array([mav1.uavPosGPSHome])))
    # uavGlobalPos = np.vstack((uavGlobalPos, np.array([mav1.uavGlobalPos])))

    # if np.linalg.norm(np.array([5, 0, 0]) - np.array(mav1.uavPosNED)) < 0.2 and mav1.uavTimeStmp > 10:
    #     # 保存数据
    #     if flag3 == False:
    #         flag3 = True

    #         path = "E:\\研究生学习\\水下机器人\\uuv20240309\\position_ctrl_data\\circle\\"

    #         np.savetxt(path + "uavTimeStmp.csv", uavTimeStmp, delimiter=',')
    #         np.savetxt(path + "trueTimeStmp.csv", trueTimeStmp, delimiter=',')

    #         np.savetxt(path + "uavAngEular.csv", uavAngEular, delimiter=',')
    #         np.savetxt(path + "trueAngEular.csv", trueAngEular, delimiter=',')

    #         np.savetxt(path + "uavAngRate.csv", uavAngRate, delimiter=',')
    #         np.savetxt(path + "trueAngRate.csv", trueAngRate, delimiter=',')

    #         np.savetxt(path + "uavPosNED.csv", uavPosNED, delimiter=',')
    #         np.savetxt(path + "truePosNED.csv", truePosNED, delimiter=',')

    #         np.savetxt(path + "uavVelNED.csv", uavVelNED, delimiter=',')
    #         np.savetxt(path + "trueVelNED.csv", trueVelNED, delimiter=',')

    #         np.savetxt(path + "uavAngQuatern.csv", uavAngQuatern, delimiter=',')
    #         np.savetxt(path + "trueAngQuatern.csv", trueAngQuatern, delimiter=',')

    #         np.savetxt(path + "uavMotorRPMS.csv", uavMotorRPMS, delimiter=',')
    #         np.savetxt(path + "trueMotorRPMS.csv", trueMotorRPMS, delimiter=',')

    #         np.savetxt(path + "uavAccB.csv", uavAccB, delimiter=',')
    #         np.savetxt(path + "trueAccB.csv", trueAccB, delimiter=',')

    #         np.savetxt(path + "uavGyro.csv", uavGyro, delimiter=',')
    #         np.savetxt(path + "uavMag.csv", uavMag, delimiter=',')
    #         np.savetxt(path + "uavVibr.csv", uavVibr, delimiter=',')
    #         np.savetxt(path + "uavPosGPS.csv", uavPosGPS, delimiter=',')
    #         np.savetxt(path + "uavPosGPSHome.csv", uavPosGPSHome, delimiter=',')
    #         np.savetxt(path + "uavGlobalPos.csv", uavGlobalPos, delimiter=',')

    #         print("完成！")
    #         break

