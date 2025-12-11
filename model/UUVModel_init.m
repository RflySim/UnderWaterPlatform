%InitFunction
load MavLinkStruct;
%Initial condition

% 飞机的初始位置
ModelInit_PosE=[0,0,0];
ModelInit_VelB=[0,0,0];
ModelInit_AngEuler=[0,0,0];
ModelInit_RateB=[0,0,0];
ModelInit_Inputs = [0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0];

%Model Parameter
% UAV TYPE.
ModelParam_uavType = int16(600); %这里是水下无人机，与三维模型的ClassID对应
ModelParam_uavMotNumbs = int8(8); % 电机数量  8

ModelParam_uavMass=11.0;%%1.5; %飞机质量
ModelParam_uavR=0.225; %飞机轴距半径
ModelParam_uavJxx = 0.1;%%2.11E-2;%转动惯量 0.0211
ModelParam_uavJyy = 0.1;%%2.19E-2; 0.0219
ModelParam_uavJzz = 0.1;%%3.66E-2; 0.0366
ModelParam_uavJ= [ModelParam_uavJxx,0,0;0,ModelParam_uavJyy,0;0,0,ModelParam_uavJzz];

% UUV机体参数结构体
UUV.buoyancy_compensation=1.0;  % 1.0
UUV.buoyancy_height_scale_limit=0.05;   % 0.05
UUV.buoyancy_origin=[0;0;0];   % 0 0 0
UUV.dampingLinear=[5.39;17.36;17.36];
UUV.dampingAngular=[0.00114;0.007;0.007];
UUV.addedMassLinear=[1.11;2.8;2.8]; %[1.11;2.8;2.8];  [0.0;0.0;0.0];
UUV.addedMassAngular=[0.00451;0.0163;0.0163]; %[0.00451;0.0163;0.0163];

% 电调参数模板
ESCTmp.isEnable=false; %是否启用本电调，默认不启用
ESCTmp.input_offset = 0;
ESCTmp.input_scaling = 1.0;
ESCTmp.zero_position_disarmed = 0;
ESCTmp.zero_position_armed = 0;
ESCTmp.joint_control_type=0;%0:velocity,1:position,...

ESC=[];%电调的数据结构体
for i=1:8 %电调集合模版最多支持8个电调
    if i<=ModelParam_uavMotNumbs %四旋翼的话，这里是4
        tmp=ESCTmp; %复制一份模版
        tmp.isEnable=true;%启用电调模型
        ESC=[ESC,tmp];%扩充维度
    else
        ESC=[ESC,ESCTmp]; %直接用默认模版填充，不启用电调模型
    end
end


% 电机通用模板
motorTmp.isEnable=false; %是否启用本电机，默认不启用
motorTmp.input_index = 1;
motorTmp.turningDirection=1;
motorTmp.pose=[-0.13 -0.2 0.023 0 0 0];

motorTmp.timeConstantUp = 0.0125;
motorTmp.timeConstantDown = 0.025;
motorTmp.maxRotVelocity = 1100;
motorTmp.motorConstant = 5.84e-06;   % UUV的这个参数，不同电机不同
motorTmp.momentConstant = 0.06;   % UUV的这个参数，不同电机不同
motorTmp.rotorDragCoefficient = 0.0;
motorTmp.rollingMomentCoefficient = 0.0;
motorTmp.rotorVelocitySlowdownSim=0.025;   % 0.025
motorTmp.reversible=1;

% pose = [...      % gazebo中的参数，yz取反
%     [0.14 0.10 0 0 -1.570796 -0.78539815];...
%     [0.14 -0.10 0 0 -1.570796 0.78539815];...
%     [-0.14 0.10 0 0 -1.570796 -2.356194];...
%     [-0.14 -0.10 0 0 -1.570796 2.356194];...
%     [0.12 0.22 -0.06 0 0 0];...
%     [0.12 -0.22 -0.06 0 0 0];...
%     [-0.12 0.22 -0.06 0 0 0];...
%     [-0.12 -0.22 -0.06 0 0 0];...
%     ];
pose = [...      % gazebo中的参数
    [0.14 -0.10 0 0 1.570796 0.78539815];...
    [0.14 0.10 0 0 1.570796 -0.78539815];...
    [-0.14 -0.10 0 0 1.570796 2.356194];...
    [-0.14 0.10 0 0 1.570796 -2.356194];...
    [0.12 -0.22 0.06 0 0 0];...
    [0.12 0.22 0.06 0 0 0];...
    [-0.12 -0.22 0.06 0 0 0];...
    [-0.12 0.22 0.06 0 0 0];...
    ];
% 电机转向  顺（CW）：-1  逆（CCW）：1
turningDirection = [1; 1; 1; 1; 1; 1; 1; 1];
motorConstant = [1; 1; -1; -1; 1; -1; -1; 1] * 10;
momentConstant = [1; 1; -1; -1; 1; -1; -1; 1] * 0.01;
motor=[];%电调的数据结构体
for i=1:8 %电调集合模版最多支持8个电调
    if i<=ModelParam_uavMotNumbs %四旋翼的话，这里是4
        tmp=motorTmp; %复制一份模版
        tmp.isEnable=true;%启用电调模型
        tmp.input_index=i-1;
        tmp.pose=pose(i,:); %设置电机位置
        tmp.turningDirection=turningDirection(i); %设置电机转向
        tmp.motorConstant = motorConstant(i);
        tmp.momentConstant = momentConstant(i);
        motor=[motor,tmp];%扩充维度
    else
        motor=[motor,motorTmp]; %直接用默认模版填充，不启用电调模型
    end
end


%-----------------------------------------
%ModelParam_uavCtrlEn = int8(0);
%ModelParam_ControlMode = int8(1); %整型 1表示Auto模式，0表示Manual模式
ModelParam_motorMinThr=0.05;
ModelParam_motorCr=842.1;
ModelParam_motorWb=22.83;
ModelParam_motorT= 0.0214;%0.0261;
ModelParam_motorJm =0.0001287;
ModelParam_rotorCm=2.783e-07;
ModelParam_rotorCt=1.681e-05;

ModelParam_noisePowerIMU=0.0003394;
ModelParam_noisePowerMag=0.0004;
ModelParam_timeSampBaro = 0.02;
ModelParam_noiseSampleTimeMag = 0.01;

ModelParam_uavCd = 0.055;
ModelParam_uavCCm = [0.0035 0.0039 0.0034];
ModelParam_uavDearo = 0.12;%%unit m

ModelParam_GlobalNoiseGainSwitch =0.4;

%Environment Parameter
ModelParam_envLongitude = 116.259368300000;
ModelParam_envLatitude = 40.1540302;
ModelParam_GPSLatLong = [ModelParam_envLatitude ModelParam_envLongitude];
ModelParam_envAltitude = -600;     %参考高度，负值


%SimParam.timeACC=1;
%SimParam.timeStep=0.0001;
% SimParam.sonarSamp=0.1;
% SimParam.AngEularSamp=0.004;
% SimParam.AngRateSamp=0.004;
% SimParam.AccSensSamp=0.004;
% SimParam.GyroSensSamp=0.004;
% SimParam.AngQuaternSamp=0.004;
% SimParam.BaroSamp=0.008;
% SimParam.AngEulerSamp=0.004;
%ModelParam_timeSampBaro = 0.01;             %我注释掉了
ModelParam_timeSampTurbWind = 0.01;
ModelParam_BusSampleRate = 0.001;



%%%ModelParam_BattModelEnable=int8(0);
%ModelParam_BattAuxCurrent=0.5;
%ModelParam_BattCells=3;
%ModelParam_BattCapacity=0.55;   %%这一项从模型配置界面传输给定
ModelParam_BattHoverMinutes=18;
ModelParam_BattHoverThr=0.609;

%GPS Parameter
ModelParam_GPSEphFinal=0.3;
ModelParam_GPSEpvFinal=0.4;
ModelParam_GPSFix3DFix=3;
ModelParam_GPSSatsVisible=10;


%Noise Parameter
ModelParam_noisePowerAccel = [0.001,0.001,0.003];%顺序 xyz 下同  不要修改这里
ModelParam_noiseSampleTimeAccel = 0.001;
%ModelParam_noiseLowPassFilterCoeAccel = 0.0001;
ModelParam_noisePowerOffGainAccel = 0.04;
ModelParam_noisePowerOffGainAccelZ = 0.03;
ModelParam_noisePowerOnGainAccel = 0.8;
ModelParam_noisePowerOnGainAccelZ = 4.5;


ModelParam_noisePowerGyro = [0.00001,0.00001,0.00001];%不要修改这里
ModelParam_noiseSampleTimeGyro = 0.001;
%ModelParam_noiseLowPassFilterCoeGyro = 0.0001;
ModelParam_noisePowerOffGainGyro = 0.02;
ModelParam_noisePowerOffGainGyroZ = 0.025;
ModelParam_noisePowerOnGainGyro = 2;%3.2;
ModelParam_noisePowerOnGainGyroZ = 1;



%ModelParam_noisePowerMag = [0.00001,0.00001,0.00001];%不要修改这里  我注释掉了
%ModelParam_noiseSampleTimeMag = 0.01;          我注释掉了
%ModelParam_noiseLowPassFilterCoeMag = 0.02;%暂时没有使用
ModelParam_noisePowerOffGainMag = 0.02;
ModelParam_noisePowerOffGainMagZ = 0.035;
ModelParam_noisePowerOnGainMag = 0.025;
ModelParam_noisePowerOnGainMagZ = 0.05;



%ModelParam_noisePowerIMU=0;%IMU噪声，这里是白噪声，这里是经过归一化  我注释掉了

ModelParam_noiseUpperGPS=0.5;  %GPS定位误差噪声，均匀噪声，这里填x,y,z的波动上限，单位是m
ModelParam_noiseGPSSampTime=0.2;%默认0.05

ModelParam_noiseUpperBaro=0; %气压计噪声，均匀噪声，这里填高度的波动上限，单位是m
ModelParam_noiseBaroSampTime=0.5;%气压计噪声更新频率，，默认0.05
ModelParam_noiseBaroCoupleWithSpeed=0;%气压计测量高度与动压关系，也就是风速与气压计掉高模型的系数，当前参数0.008飞机10m/s掉高1m

ModelParam_noiseUpperWindBodyRatio=0;%风波动系数，风速*(1+该系数)
ModelParam_noiseWindSampTime=0.001;


%%ModelParam_envGravityAcc = 9.81;
ModelParam_envAirDensity = 1.225;    %还没有用到
ModelParam_envDiffPressure = 0; % Differential pressure (airspeed) in millibar
ModelParam_noiseTs = 0.001;

%ModelParam_FailModelStartT = 5;
%ModelParam_FailModelLastT = 5;