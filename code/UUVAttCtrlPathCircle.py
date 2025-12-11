# import required libraries
import time
import math
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

    mav1.SendAttPX4([roll, pitch, yaw], thrust)
    
    yaw += 0.1

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

    # if yaw >= 360:
    #     # 保存数据
    #     if flag3 == False:
    #         flag3 = True

    #         path = "E:\\研究生学习\\水下机器人\\uuv20240309\\new_model_data\\attitude_ctrl_data\\circle\\"

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

