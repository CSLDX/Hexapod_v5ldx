%% 六足机器人的节律运动步态(3/4/5足步态)
%**************************************************************************
%*****************【Authors：AndyGao；Date：2020/11】**********************
% 为了模块化设计，设置机器人在每个步幅结束后都恢复到初始位形
%**************************************************************************
clear;    clc;

%% 机器人结构参数
global BR L1 L2 L3 L4 Alpha
% 机身半径R，腿部各连杆参数L，吸盘半径r （单位：m）
L1 = 0.09;    L2 = 0.15;    L3 = 0.16;    L4 = 0.15;    BR = 0.18;
% 六条支链分布角 （单位：rad）
Alpha = [pi/6, pi/2, 5*pi/6, -5*pi/6, -pi/2, -pi/6];

%% 机器人的初始位型参数
global Theta_0 beta_0 phi_0
Theta_0 = zeros(6,4);   % 各关节角度初值 （单位：rad）
beta_0 = (pi/180)*(0);  % 吸盘的相对初始转角 （单位：rad）
phi_0 = (pi/180)*(0);   % 机体平台的初始俯仰角 （单位：rad）

%% 设置运动参数
global liftH Xstride Ystride Zrotate
liftH = 0.10;           % 摆动相的抬腿高度(LiftH)（单位：m）
% 机体质心(在三个自由方向)的期望步幅(Stride)
Xstride = input('***提示*** 请输入机体质心沿着 X轴 方向的期望步幅Xstride（单位：m）：');
Ystride = input('***提示*** 请输入机体质心沿着 Y轴 方向的期望步幅Ystride（单位：m）：');
Zrotate = input('***提示*** 请输入机体质心绕着 Z轴 自旋的期望步幅Zrotate（单位：deg）：') * (pi/180);

% 摆动相-支撑相标志符（1：摆动相；-1：支撑相）
bool_flag = 0;
while (bool_flag == 0)
    gait_flag = input('***提示*** 请确定机器人的步态模式【3/4/5 足？】：');
    if (gait_flag == 3)
        SwiSur = [1 -1 1 -1 1 -1
                  -1 1 -1 1 -1 1];
        bool_flag = 1;
    elseif (gait_flag == 4)
        SwiSur = [1 -1 -1 1 -1 -1
                  -1 1 -1 -1 1 -1
                  -1 -1 1 -1 -1 1];
        bool_flag = 1;
    elseif (gait_flag == 5)
        SwiSur = [1 -1 -1 -1 -1 -1
                  -1 1 -1 -1 -1 -1
                  -1 -1 1 -1 -1 -1
                  -1 -1 -1 1 -1 -1
                  -1 -1 -1 -1 1 -1
                  -1 -1 -1 -1 -1 1];
        bool_flag = 1;
    else
        bool_flag = 0;
    end
end
[row, ~] = size(SwiSur);
duty = gait_flag / 6;   % 占空比

%% 【单摆动周期内，运动参数设置】
swigTend = 1;           % 单摆动周期时长【可自定义】
swigNum = 100;          % 单摆动周期插值点数【勿改动，否则vrep_main中也需作相应修改】
SwigTimeSeq = linspace(0,swigTend, swigNum); % 单摆动周期的时间序列
% 【单摆动周期内的Sigmoid型插值函数】
const = 20;             % Sigmoid插值参数
Tg = swigTend / 2;      % Sigmoid插值中的给定时间参数
Gamma = 1 ./ (1 + exp(-const .* (SwigTimeSeq - Tg)));	% Sigmoid函数

%% 【单步幅周期内，足端(相对于机体系)的位移】
% 【变量存储矩阵】
StriRelFootPose = cell(6,1);	% 【单步幅】足端相对于机体系的位置 [srFx;srFy;srFz]
StriTheta = cell(6,1);          % 【单步幅】关节角度矩阵 [q1;q2;q3;q4]
StriMovRate = cell(6,1);        % 【单步幅】单步幅周期内，各足的瞬时运动步幅比例
StriRelBodyPose = zeros(4,row*swigNum); % 【单步幅】机心相对于机体系的位姿 [srBx;srBy;srBz;srByaw]

r = 0;
while (r < row)
    r = r + 1;
    % 当前摆动周期内，腿部的摆动/支撑状态
    if (mod(r,row) ~= 0)
        SwiSur_cur = SwiSur(mod(r,row), :);
    else
        SwiSur_cur = SwiSur(end, :);
    end
    
for L = 1 : 6
    alphaL = Alpha(L); % 当前支链的方位角
    % 足端相对于机体系的初始位置
    Fx_0 = BR*cos(alphaL) + (L1 + L2*cos(Theta_0(L,2)) + L3*sin(Theta_0(L,2)+Theta_0(L,3)) +...
           L4*sin(Theta_0(L,2)+Theta_0(L,3)+Theta_0(L,4))) * cos(alphaL + Theta_0(L,1));
    Fy_0 = BR*sin(alphaL) + (L1 + L2*cos(Theta_0(L,2)) + L3*sin(Theta_0(L,2)+Theta_0(L,3)) +...
           L4*sin(Theta_0(L,2)+Theta_0(L,3)+Theta_0(L,4))) * sin(alphaL + Theta_0(L,1));
    Fz_0 = L2*sin(Theta_0(L,2)) - L3*cos(Theta_0(L,2)+Theta_0(L,3)) - L4*cos(Theta_0(L,2)+Theta_0(L,3)+Theta_0(L,4));
    
    % 各足的瞬时运动步幅比例初值
    StriMovRate{L}(1,1) = 0;
    % 腿部状态的符号标签：摆动相为1，支撑相为-1
    sgnL = SwiSur_cur(L);
    % 单摆动周期内，摆动相/支撑相的运动步幅总比例（代数值）
    swigMovRate = (1+sgnL)/2 * sgnL * duty + (1-sgnL)/2 * sgnL * (1 - duty); % 前者对应摆动相，后者对应支撑相
    
    for k = 1 : swigNum
        kk = k + (r - 1) * swigNum;
        %--------------------------------------------------------------------------
        % 单步幅周期内，足端的笛卡尔位置
        %--------------------------------------------------------------------------
        StriMovRate{L}(1,kk) = StriMovRate{L}(1,max(1,(r-1)*swigNum)) + swigMovRate * Gamma(k); % 各足的瞬时运动步幅比例
        % 足端相对于机体系的瞬时位置坐标
        StriRelFootPose{L}(1,kk) = Fx_0 * cos(Zrotate*StriMovRate{L}(1,kk)) - Fy_0 * sin(Zrotate*StriMovRate{L}(1,kk)) + Xstride*StriMovRate{L}(1,kk);
        StriRelFootPose{L}(2,kk) = Fx_0 * sin(Zrotate*StriMovRate{L}(1,kk)) + Fy_0 * cos(Zrotate*StriMovRate{L}(1,kk)) + Ystride*StriMovRate{L}(1,kk);
        StriRelFootPose{L}(3,kk) = Fz_0 + (1+sgnL)/2 * liftH * (-0.5 * cos(2*pi/swigTend * SwigTimeSeq(k)) + 0.5);
        
        %--------------------------------------------------------------------------
        % 单步幅周期内，运动学逆解
        %--------------------------------------------------------------------------
        Fx_cur = StriRelFootPose{L}(1,kk); % 当前时刻的足端横坐标
        Fy_cur = StriRelFootPose{L}(2,kk); % 当前时刻的足端纵坐标
        Fz_cur = StriRelFootPose{L}(3,kk); % 当前时刻的足端竖坐标
        phi_i  = phi_0;                 % 机体平台的瞬时俯仰角
        beta_i = beta_0;                % 吸盘的转角
        % 当前时刻的关节变量
        theta1 = atan2(Fy_cur - BR*sin(alphaL), Fx_cur - BR*cos(alphaL)) - alphaL;
        tempA = Fz_cur + L4*cos(beta_i-sign(alphaL)*phi_i);
        tempB = (Fy_cur - BR*sin(alphaL))/sin(alphaL+theta1) - L1 - L4*sin(beta_i-sign(alphaL)*phi_i);
        tempC = L2;
        tempD = -L3;
        theta3 = asin((tempA^2+tempB^2-tempC^2-tempD^2) / (-2*tempC*tempD));
        theta2 = atan((-2*tempC*tempD*cos(theta3)) / (tempA^2+tempB^2+tempC^2-tempD^2)) + atan2(tempA , tempB);
        theta4 = beta_i - sign(alphaL)*phi_i - theta2 - theta3;
        % 存储关节变量
        Theta_ik = [theta1; theta2; theta3; theta4];
        StriTheta{L} = [StriTheta{L}, Theta_ik]; % 单步幅内各关节角度值
        
        %--------------------------------------------------------------------------
        % 单步幅周期内，机心的相对位姿
        %--------------------------------------------------------------------------
        StriRelBodyPose(4,kk) = StriRelBodyPose(4,max(1,(r-1)*swigNum)) + Zrotate / 2 * Gamma(k);
        StriRelBodyPose(1:3,kk) = StriRelBodyPose(1:3,max(1,(r-1)*swigNum)) + rotz(Zrotate / 2 * Gamma(k)) * ( [Xstride;Ystride;0] / 2 * Gamma(k) );
        
    end
end
end

%% 多步幅连续节律运动
% 节律运动参数设置
rhyNum = 1;                    % 周期节律运动的步幅数【可自定义】
RhyRelFootPose = cell(6,1);     % 节律运动的足端相对位姿 % [rrFx;rrFy;rrFz]
RhyAbsFootPose = cell(6,1);     % 节律运动的足端绝对位姿 % [raFx;raFy;raFz]
RhyTheta = cell(6,1);           % 节律运动的关节位移 [q1;q2;q3;q4]
RhyRelBodyPose = [];            % 节律运动的机心相对于机体系的位姿 [rrBx;rrBy;rrBz;rrByaw]
RhyAbsBodyPose = [];            % 节律运动的机心相对于世界系的绝对位姿 [raBx;raBy;raBz;raByaw]
RhyTimeSeq = linspace(0,row*swigTend*rhyNum, row*swigNum*rhyNum); % 节律运动的仿真时间

% 节律运动的相对位移
for n = 1 : rhyNum
    for L = 1 : 6
        RhyRelFootPose{L} = [RhyRelFootPose{L}, StriRelFootPose{L}];	% [rrFx;rrFy;rrFz]
        RhyTheta{L} = [RhyTheta{L}, StriTheta{L}];                      % [q1;q2;q3;q4]
    end
    RhyRelBodyPose = [RhyRelBodyPose, StriRelBodyPose];                 % [rrBx;rrBy;rrBz;rrByaw]
end

%% 绘图
for L = 1 : 6
    figure(L);    clf;
    hold on;    box on;
    plot(RhyTimeSeq, RhyTheta{L}(1,:), 'k:','LineWidth',2);
    plot(RhyTimeSeq, RhyTheta{L}(2,:), 'r:','LineWidth',2);
    plot(RhyTimeSeq, RhyTheta{L}(3,:), 'g:','LineWidth',2);
    plot(RhyTimeSeq, RhyTheta{L}(4,:), 'b:','LineWidth',2);
    plot(RhyTimeSeq, RhyRelFootPose{L}(1,:), '-r','LineWidth',3);
    plot(RhyTimeSeq, RhyRelFootPose{L}(2,:), '-g','LineWidth',3);
    plot(RhyTimeSeq, RhyRelFootPose{L}(3,:), '-b','LineWidth',3);
end
% %% 存储各关节角度值
% JointAngles = RhyTheta;
% save(['Datas/Gait',num2str(gait_flag),'_JointAngles'], 'JointAngles');
% %% 存储各腿位置变化值
% FootPose = RhyRelFootPose;
% save(['Datas/Gait',num2str(gait_flag),'_FootPose'], 'FootPose');

disp('***** 执行完毕！*****');
